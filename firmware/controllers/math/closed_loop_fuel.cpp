/**
 * @file closed_loop_fuel.cpp
 * @brief Closed-loop fuel correction (short-term fuel trim).
 *
 * Computes a fueling correction from O2/lambda sensor feedback versus the target
 * lambda, optionally split into independent cells (by RPM/load) so each operating
 * region learns its own short-term trim.
 */

#include "pch.h"

#include "closed_loop_fuel.h"
#include "tunerstudio.h"

#if EFI_ENGINE_CONTROL

SensorType ShortTermFuelTrim::getSensorForBankIndex(size_t index) {
	switch (index) {
		case 0: return SensorType::Lambda1;
		case 1: return SensorType::Lambda2;
		default: return SensorType::Invalid;
	}
}

ft_region_e ShortTermFuelTrim::computeStftBin(float rpm, float load, stft_s& cfg) {
	// Low RPM -> idle
	if (idleDeadband.lt(rpm, cfg.maxIdleRegionRpm))
	{
		return ftRegionIdle;
	}

	// Low load -> overrun
	if (overrunDeadband.lt(load, cfg.maxOverrunLoad))
	{
		return ftRegionOverrun;
	}

	// High load -> power
	if (loadDeadband.gt(load, cfg.minPowerLoad))
	{
		return ftRegionPower;
	}

	// Default -> normal "in the middle" cell
	return ftRegionCruise;
}

stft_state_e ShortTermFuelTrim::getCorrectionState() {
	const auto& cfg = engineConfiguration->stft;

	// User disable bit
	if (!engineConfiguration->fuelClosedLoopCorrectionEnabled) {
		return stftDisabledSettings;
	}

	// Don't correct if tuning seems to be happening
	if (checkIfTuningVeNow()) {
		return stftDisabledTuning;
	}

	// Don't correct if not running
	if (!engine->rpmCalculator.isRunning()) {
		return stftDisabledRPM;
	}

	// Startup delay - allow O2 sensor to warm up, etc
	if (cfg.startupDelay > engine->fuelComputer.running.timeSinceCrankingInSecs) {
		return stftDisabledCrankingDelay;
	}

	// Check that the engine is hot enough (and clt not failed)
	auto clt = Sensor::get(SensorType::Clt);
	if (!clt.Valid || clt.Value < cfg.minClt) {
		return stftDisabledClt;
	}

	// If all was well, then we're enabled!
	return stftEnabled;
}

stft_state_e ShortTermFuelTrim::getLearningState(SensorType sensor) {
	const auto& cfg = engineConfiguration->stft;

	// TODO: add check for stftLearningDisabledSettings

	// Pause (but don't reset) correction if the AFR is off scale.
	// It's probably a transient and poorly tuned transient correction
	// TODO: use getStoichiometricRatio() instead of STOICH_RATIO
	auto afr = Sensor::getOrZero(sensor) * STOICH_RATIO;
	if (!afr || afr < cfg.minAfr || afr > cfg.maxAfr) {
		return stftDisabledAfrOurOfRange;
	}

	// Pause correction if DFCO was active recently
	auto timeSinceDfco = engine->module<DfcoController>()->getTimeSinceCut();
	if (timeSinceDfco < engineConfiguration->noFuelTrimAfterDfcoTime) {
		return stftDisabledDFCO;
	}

	// Pause correction if Accel enrichment was active recently
	auto timeSinceAccel = engine->module<TpsAccelEnrichment>()->getTimeSinceAcell();
	if (timeSinceAccel < engineConfiguration->noFuelTrimAfterAccelTime) {
		return stftDisabledTpsAccel;
	}

	// Pause if some other cut was active recently
	auto timeSinceFuelCut = engine->module<LimpManager>()->getTimeSinceAnyCut();
	// TODO: should duration this be configurable?
	if (timeSinceFuelCut < 2) {
		return stftDisabledFuelCut;
	}

	return stftEnabled;
}

void ShortTermFuelTrim::init(stft_s *stftCfg) {
	for (size_t bank = 0; bank < FT_BANK_COUNT; bank++) {
		for (size_t bin = 0; bin < STFT_CELL_COUNT; bin++) {
			auto& cell = banks[bank].cells[bin];
			SensorType sensor = getSensorForBankIndex(bank);

			cell.configure(&stftCfg->cellCfgs[bin], sensor);
		}
	}
}

// Periodic step mode: bounds for the correction period read from the curve, so a
// zeroed or misconfigured curve can not make the loop step arbitrarily fast.
static constexpr float MIN_CORRECTION_PERIOD_MS = 100.0f;
static constexpr float MAX_CORRECTION_PERIOD_MS = 5000.0f;
// Only the tail of the window reflects the settled response to the previous step,
// so only that part is averaged into the error used for the next step.
static constexpr float STEP_TAIL_FRACTION = 0.75f;
// Minimum number of tail samples before a step may fire, in case learning was
// paused (DFCO, accel, ...) for part of the window.
static constexpr uint16_t STEP_MIN_SAMPLES = 5;

ClosedLoopFuelResult ShortTermFuelTrim::getCorrection(float rpm, float fuelLoad) {
	stftCorrectionState = getCorrectionState();
	if (stftCorrectionState != stftEnabled) {
		// Learning is also prohibited
		for (size_t bank = 0; bank < FT_BANK_COUNT; bank++) {
			stftLearningState[bank] = stftCorrectionState;
		}
		return {};
	}

	const auto& cfg = engineConfiguration->stft;

	ClosedLoopFuelResult result;

	result.region = stftCorrectionBinIdx = computeStftBin(rpm, fuelLoad, engineConfiguration->stft);

	bool periodicStep = cfg.correctionAlgorithm == StftAlgo_PeriodicStep;
	float periodMs = 0;
	if (periodicStep) {
		periodMs = interpolate2d(engine->engineState.airflowEstimate, cfg.correctionPeriodFlowBins, cfg.correctionPeriodMs);
		periodMs = clampF(MIN_CORRECTION_PERIOD_MS, periodMs, MAX_CORRECTION_PERIOD_MS);

		// A step is only valid against readings from the current cell, so start a
		// fresh observation window whenever the region changes.
		if (result.region != m_lastStepRegion) {
			m_lastStepRegion = result.region;
			for (size_t bank = 0; bank < FT_BANK_COUNT; bank++) {
				m_stepErrorSum[bank] = 0;
				m_stepErrorCount[bank] = 0;
				m_stepTimer[bank].reset();
			}
		}
	}
	stftPeriodMs = (uint16_t)periodMs;

	for (size_t bank = 0; bank < FT_BANK_COUNT; bank++) {
		auto& cell = banks[bank].cells[stftCorrectionBinIdx];

		SensorType sensor = getSensorForBankIndex(bank);

		stftLearningState[bank] = getLearningState(sensor);
		if (stftLearningState[bank] == stftEnabled) {
			stftInputError[bank] = cell.getLambdaError();
			if (periodicStep) {
				updatePeriodicStep(bank, cell, periodMs);
			} else {
				cell.update(PERCENT_DIV * cfg.deadband, engineConfiguration->stftIgnoreErrorMagnitude);
			}
			stftLearningBinIdx = stftCorrectionBinIdx;
		}

		result.banks[bank] = cell.getAdjustment();
	}

	return result;
}

void ShortTermFuelTrim::updatePeriodicStep(size_t bank, ClosedLoopFuelCellBase& cell, float periodMs) {
	const auto& cfg = engineConfiguration->stft;

	float elapsedMs = m_stepTimer[bank].getElapsedSeconds() * 1000.0f;

	// Average the error over the settled tail of the window instead of trusting
	// a single reading - this filters sensor noise for free.
	if (elapsedMs >= STEP_TAIL_FRACTION * periodMs) {
		m_stepErrorSum[bank] += cell.getLambdaError();
		m_stepErrorCount[bank]++;
	}

	// Wait out the full sensor response before judging the previous step
	if (elapsedMs < periodMs || m_stepErrorCount[bank] < STEP_MIN_SAMPLES) {
		return;
	}

	float avgError = m_stepErrorSum[bank] / m_stepErrorCount[bank];

	// If we're within the deadband, make no adjustment - same policy as the integrator mode.
	if (std::abs(avgError) >= PERCENT_DIV * cfg.deadband) {
		if (engineConfiguration->stftIgnoreErrorMagnitude) {
			avgError = avgError > 0 ? 0.1f : -0.1f;
		}

		float maxStep = PERCENT_DIV * cfg.maxStepPercent;
		cell.applyStep(clampF(-maxStep, cfg.trimStepGain * avgError, maxStep));
	}

	m_stepErrorSum[bank] = 0;
	m_stepErrorCount[bank] = 0;
	m_stepTimer[bank].reset();
}

void ShortTermFuelTrim::onSlowCallback() {
	// Do some magic math here?
}

bool ShortTermFuelTrim::needsDelayedShutoff() {
	return false;
}

void initStft(void)
{
	engine->module<ShortTermFuelTrim>()->init(&engineConfiguration->stft);
}

/* TODO: move out of here */
bool checkIfTuningVeNow() {
#if EFI_TUNER_STUDIO
	const bool result = isTuningVeNow();
#else
	const bool result = false;
#endif /* EFI_TUNER_STUDIO */
	engine->outputChannels.isTuningNow = result;
	return result;
}

#endif // EFI_ENGINE_CONTROL
