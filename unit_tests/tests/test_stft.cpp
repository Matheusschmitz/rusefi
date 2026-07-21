#include "pch.h"

#include "closed_loop_fuel_cell.h"
#include "closed_loop_fuel.h"

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

class MockClCell : public ClosedLoopFuelCellBase {
public:
	MOCK_METHOD(float, getLambdaError, (), (const));
	MOCK_METHOD(float, getMaxAdjustment, (), (const));
	MOCK_METHOD(float, getMinAdjustment, (), (const));
	MOCK_METHOD(float, getIntegratorGain, (), (const));
};

TEST(ClosedLoopCell, TestDeadband) {
	StrictMock<MockClCell> cl;

	// Error is more than deadtime, so nothing else should be called
	EXPECT_CALL(cl, getLambdaError())
		.WillOnce(Return(0.05f));

	cl.update(0.1f, true);

	// Should be zero adjustment
	EXPECT_FLOAT_EQ(cl.getAdjustment(), 1.0f);
}

TEST(ClosedLoopFuelCell, AdjustRate) {
	StrictMock<MockClCell> cl;

	EXPECT_CALL(cl, getLambdaError())
		.WillOnce(Return(0.1f));
	EXPECT_CALL(cl, getMinAdjustment())
		.WillOnce(Return(-0.2f));
	EXPECT_CALL(cl, getMaxAdjustment())
		.WillOnce(Return(0.2f));
	EXPECT_CALL(cl, getIntegratorGain())
		.WillOnce(Return(2.0f));

	cl.update(0.0f, false);

	// Should have integrated 0.2 * dt
	// dt = 1000.0f / FAST_CALLBACK_PERIOD_MS
	EXPECT_FLOAT_EQ(cl.getAdjustment(), 1 + (0.2f / (1000.0f / FAST_CALLBACK_PERIOD_MS)));
}

TEST(ClosedLoopFuel, CellSelection) {
	ShortTermFuelTrim stft;
	stft_s cfg;

	// Sensible region config
	cfg.maxIdleRegionRpm = 1500;
	cfg.minPowerLoad = 80;
	cfg.maxOverrunLoad = 30;

	stft.init(&cfg);

	// Test idle
	EXPECT_EQ(0u, stft.computeStftBin(1000, 10, cfg));
	EXPECT_EQ(0u, stft.computeStftBin(1000, 50, cfg));
	EXPECT_EQ(0u, stft.computeStftBin(1000, 90, cfg));

	// Test overrun
	EXPECT_EQ(1u, stft.computeStftBin(2000, 10, cfg));
	EXPECT_EQ(1u, stft.computeStftBin(4000, 10, cfg));
	EXPECT_EQ(1u, stft.computeStftBin(10000, 10, cfg));

	// Test load
	EXPECT_EQ(2u, stft.computeStftBin(2000, 90, cfg));
	EXPECT_EQ(2u, stft.computeStftBin(4000, 90, cfg));
	EXPECT_EQ(2u, stft.computeStftBin(10000, 90, cfg));

	// Main cell
	EXPECT_EQ(3u, stft.computeStftBin(2000, 50, cfg));
	EXPECT_EQ(3u, stft.computeStftBin(4000, 50, cfg));
	EXPECT_EQ(3u, stft.computeStftBin(10000, 50, cfg));
}

TEST(ClosedLoopFuel, afrLimits) {
	ShortTermFuelTrim stft;
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->stft.minAfr = 10;  // 10.0 AFR
	engineConfiguration->stft.maxAfr = 18;  // 18.0 AFR

	stft.init(&engineConfiguration->stft);

	Sensor::setMockValue(SensorType::Lambda1, 0.1f);
	EXPECT_NE(stft.getLearningState(SensorType::Lambda1), stftEnabled);

	Sensor::setMockValue(SensorType::Lambda1, 1.0f);
	EXPECT_EQ(stft.getLearningState(SensorType::Lambda1), stftEnabled);

	Sensor::setMockValue(SensorType::Lambda1, 2.0f);
	EXPECT_NE(stft.getLearningState(SensorType::Lambda1), stftEnabled);
}

// Configure an enabled STFT with a flat correction period curve and a running,
// warmed-up engine so that getCorrection() is fully active.
static void configurePeriodicStepStft(float periodMs) {
	auto& cfg = engineConfiguration->stft;

	engineConfiguration->fuelClosedLoopCorrectionEnabled = true;
	engineConfiguration->stftIgnoreErrorMagnitude = false;
	engineConfiguration->isTuningDetectorEnabled = false;

	cfg.correctionAlgorithm = StftAlgo_PeriodicStep;
	cfg.trimStepGain = 0.5f;
	cfg.maxStepPercent = 2.0f;
	cfg.deadband = 0;
	cfg.startupDelay = 0;
	cfg.minClt = 0;
	cfg.minAfr = 10;
	cfg.maxAfr = 20;
	cfg.maxIdleRegionRpm = 1000;
	cfg.maxOverrunLoad = 20;
	cfg.minPowerLoad = 90;

	for (size_t i = 0; i < STFT_PERIOD_CURVE_SIZE; i++) {
		cfg.correctionPeriodFlowBins[i] = 50.0f * i;
		cfg.correctionPeriodMs[i] = (uint16_t)periodMs;
	}

	for (size_t i = 0; i < STFT_CELL_COUNT; i++) {
		cfg.cellCfgs[i].maxAdd = 25;
		cfg.cellCfgs[i].maxRemove = 25;
	}

	engine->rpmCalculator.setRpmValue(2000);
	Sensor::setMockValue(SensorType::Clt, 80);
	engine->fuelComputer.targetLambda = 1.0f;
	engine->engineState.airflowEstimate = 50;
}

// Run one 5ms fast callback worth of STFT and advance time
static float runStftCallback(ShortTermFuelTrim& stft, float rpm, float load) {
	float adj = stft.getCorrection(rpm, load).banks[0];
	advanceTimeUs(5'000);
	return adj;
}

TEST(ClosedLoopFuel, PeriodicStepWaitsFullPeriod) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configurePeriodicStepStft(200);

	ShortTermFuelTrim stft;
	stft.init(&engineConfiguration->stft);
	Sensor::setMockValue(SensorType::Lambda1, 1.10f); // 10% lean

	// First window: a full period must elapse before the first step
	float adj = 1.0f;
	for (int i = 0; i < 40; i++) {
		adj = runStftCallback(stft, 2000, 50);
	}
	EXPECT_NEAR(adj, 1.0f, 1e-6);

	// Crossing the period boundary fires one step:
	// gain 0.5 * error 0.1 = 5%, clamped to the 2% max step
	for (int i = 0; i < 3; i++) {
		adj = runStftCallback(stft, 2000, 50);
	}
	EXPECT_NEAR(adj, 1.02f, 1e-5);

	// No further steps inside the next window...
	for (int i = 0; i < 30; i++) {
		adj = runStftCallback(stft, 2000, 50);
	}
	EXPECT_NEAR(adj, 1.02f, 1e-5);

	// ...and one more once it expires
	for (int i = 0; i < 15; i++) {
		adj = runStftCallback(stft, 2000, 50);
	}
	EXPECT_NEAR(adj, 1.04f, 1e-5);
}

TEST(ClosedLoopFuel, PeriodicStepStepIsGainTimesAvgError) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configurePeriodicStepStft(200);

	ShortTermFuelTrim stft;
	stft.init(&engineConfiguration->stft);

	// Small error: gain 0.5 * error 0.02 = 1%, below the 2% step limit
	Sensor::setMockValue(SensorType::Lambda1, 1.02f);

	float adj = 1.0f;
	for (int i = 0; i < 43; i++) {
		adj = runStftCallback(stft, 2000, 50);
	}
	EXPECT_NEAR(adj, 1.01f, 1e-4);
}

TEST(ClosedLoopFuel, PeriodicStepRegionChangeResetsWindow) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configurePeriodicStepStft(200);

	ShortTermFuelTrim stft;
	stft.init(&engineConfiguration->stft);
	Sensor::setMockValue(SensorType::Lambda1, 1.10f);

	// Take a step in the cruise cell
	float adj = 1.0f;
	for (int i = 0; i < 43; i++) {
		adj = runStftCallback(stft, 2000, 50);
	}
	EXPECT_GT(adj, 1.0f);

	// Moving to the overrun cell must start a fresh window: no step until a
	// full period has elapsed in the new region
	for (int i = 0; i < 39; i++) {
		adj = runStftCallback(stft, 2000, 10);
	}
	EXPECT_NEAR(adj, 1.0f, 1e-6);

	for (int i = 0; i < 4; i++) {
		adj = runStftCallback(stft, 2000, 10);
	}
	EXPECT_GT(adj, 1.0f);
}

// First-order-plus-dead-time lambda response to the applied trim: lambda settles
// at lambda0 - (trim - 1) once the trim has propagated through the dead time.
struct FopdtLambdaPlant {
	// Buffer capacity, not a trial length - callers pick how many of these 5ms
	// ticks to actually run. 12000 covers up to 60 simulated seconds, enough to
	// watch a slow (30s time constant) integrator run to completion.
	static constexpr int MAX_STEPS = 12000;
	static constexpr float DT = 0.005f;

	float lambda0;
	float deadTimeSec;
	float tauSec;
	float sensorLambda;
	float trimHistory[MAX_STEPS];
	int step = 0;

	FopdtLambdaPlant(float lambda0, float deadTimeSec, float tauSec)
		: lambda0(lambda0), deadTimeSec(deadTimeSec), tauSec(tauSec), sensorLambda(lambda0) {}

	// Advance the plant by one 5ms tick with the currently applied trim
	void tick(float trim) {
		trimHistory[step] = trim;
		int delaySteps = (int)(deadTimeSec / DT);
		float delayedTrim = step >= delaySteps ? trimHistory[step - delaySteps] : 1.0f;
		float settled = lambda0 - (delayedTrim - 1.0f);
		sensorLambda += (settled - sensorLambda) * (DT / tauSec);
		step++;
	}
};

TEST(ClosedLoopFuel, PeriodicStepConvergesWithoutOvershoot) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configurePeriodicStepStft(300);
	// Allow larger steps so convergence takes only a few periods
	engineConfiguration->stft.maxStepPercent = 5.0f;

	ShortTermFuelTrim stft;
	stft.init(&engineConfiguration->stft);

	// 10% lean plant, 60ms dead time + 40ms sensor lag: fully settled well
	// within the 300ms correction period
	FopdtLambdaPlant plant(1.10f, 0.060f, 0.040f);

	constexpr int trialSteps = 1000; // 5 seconds
	float trim = 1.0f;
	float minLambda = plant.sensorLambda;
	for (int i = 0; i < trialSteps; i++) {
		plant.tick(trim);
		Sensor::setMockValue(SensorType::Lambda1, plant.sensorLambda);
		trim = stft.getCorrection(2000, 50).banks[0];
		advanceTimeUs(5'000);
		if (plant.sensorLambda < minLambda) {
			minLambda = plant.sensorLambda;
		}
	}

	// Converged on target...
	EXPECT_NEAR(plant.sensorLambda, 1.0f, 0.01f);
	EXPECT_NEAR(trim, 1.10f, 0.015f);
	// ...without hunting past it
	EXPECT_GT(minLambda, 0.985f);
}

struct ConvergenceResult {
	float settleTimeSec; // -1 if it never entered the settle band within the trial
	float overshootLambda; // how far past the target the sensor swung, 0 if it never did
};

// Runs a fresh STFT instance against a fresh plant for the requested tuning: either an
// integrator time constant (seconds) or a periodic step correction period (ms), applied
// flat across the whole curve/all cells so the trial isolates the tuning knob under test.
static ConvergenceResult runConvergenceTrial(bool periodicStep, float tuningValue, int trialSteps) {
	auto& cfg = engineConfiguration->stft;
	cfg.correctionAlgorithm = periodicStep ? StftAlgo_PeriodicStep : StftAlgo_Integrator;

	if (periodicStep) {
		for (size_t i = 0; i < STFT_PERIOD_CURVE_SIZE; i++) {
			cfg.correctionPeriodMs[i] = (uint16_t)tuningValue;
		}
	} else {
		for (size_t i = 0; i < STFT_CELL_COUNT; i++) {
			cfg.cellCfgs[i].timeConstant = tuningValue;
		}
	}

	ShortTermFuelTrim stft;
	stft.init(&cfg);

	// Same plant as PeriodicStepConvergesWithoutOvershoot above: 10% lean, 60ms dead
	// time + 40ms sensor lag, fully settled well inside the shortest period tried here.
	FopdtLambdaPlant plant(1.10f, 0.060f, 0.040f);

	constexpr float target = 1.0f;
	constexpr float settleBand = 0.005f;

	float trim = 1.0f;
	float minLambda = plant.sensorLambda;
	float settleTimeSec = -1;

	for (int i = 0; i < trialSteps; i++) {
		plant.tick(trim);
		Sensor::setMockValue(SensorType::Lambda1, plant.sensorLambda);
		trim = stft.getCorrection(2000, 50).banks[0];
		advanceTimeUs(5'000);

		if (plant.sensorLambda < minLambda) {
			minLambda = plant.sensorLambda;
		}
		if (settleTimeSec < 0 && std::abs(plant.sensorLambda - target) < settleBand) {
			settleTimeSec = i * FopdtLambdaPlant::DT;
		}
	}

	return { settleTimeSec, std::max(0.0f, target - minLambda) };
}

// Head-to-head comparison of the integrator at several time constants against periodic
// step at a couple of correction periods, all against the identical plant. Prints a
// table so the numbers are visible directly in the test log, and asserts the headline
// claim: periodic step settles dramatically faster than the integrator, with no more
// overshoot.
TEST(ClosedLoopFuel, CompareIntegratorTuningsVsPeriodicStep) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configurePeriodicStepStft(300); // baseline STFT/engine setup; tuning overridden per trial
	engineConfiguration->stft.maxStepPercent = 5.0f;

	constexpr int trialSteps = 12000; // 60 simulated seconds

	struct Trial { const char* name; bool periodicStep; float tuning; };
	Trial trials[] = {
		{ "Integrator tau=5s",             false,   5.0f },
		{ "Integrator tau=15s",            false,  15.0f },
		{ "Integrator tau=30s (default)",  false,  30.0f },
		{ "Periodic step T=300ms",         true,  300.0f },
		{ "Periodic step T=600ms",         true,  600.0f },
	};

	printf("\n%-32s %12s %14s\n", "Configuration", "Settle (s)", "Overshoot (%)");
	printf("--------------------------------------------------------------------\n");

	ConvergenceResult results[efi::size(trials)];
	for (size_t i = 0; i < efi::size(trials); i++) {
		results[i] = runConvergenceTrial(trials[i].periodicStep, trials[i].tuning, trialSteps);

		if (results[i].settleTimeSec < 0) {
			printf("%-32s %12s %13.2f%%\n", trials[i].name, "> 60.0", results[i].overshootLambda * 100.0f);
		} else {
			printf("%-32s %12.2f %13.2f%%\n", trials[i].name, results[i].settleTimeSec, results[i].overshootLambda * 100.0f);
		}

		// Whatever the tuning, neither algorithm should overshoot past the target here -
		// that would signal a tuning that is too aggressive for this plant.
		EXPECT_LT(results[i].overshootLambda, 0.02f) << trials[i].name;
	}
	printf("\n");

	// Headline claim: periodic step at a representative period settles several times
	// faster than the default integrator tuning against the same plant.
	auto& periodicDefault = results[3]; // T=300ms
	auto& integratorDefault = results[2]; // tau=30s
	ASSERT_GE(periodicDefault.settleTimeSec, 0.0f);
	if (integratorDefault.settleTimeSec >= 0.0f) {
		EXPECT_LT(periodicDefault.settleTimeSec * 5, integratorDefault.settleTimeSec);
	} else {
		// Integrator at tau=30s isn't even expected to settle within the 60s trial -
		// that alone demonstrates the gap.
		SUCCEED();
	}
}

// Pin the integrator mode behavior through getCorrection(): the periodic step
// additions must leave the existing algorithm untouched.
TEST(ClosedLoopFuel, IntegratorModeViaGetCorrection) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configurePeriodicStepStft(200);
	auto& cfg = engineConfiguration->stft;
	cfg.correctionAlgorithm = StftAlgo_Integrator;
	cfg.cellCfgs[ftRegionCruise].timeConstant = 10;

	ShortTermFuelTrim stft;
	stft.init(&engineConfiguration->stft);
	Sensor::setMockValue(SensorType::Lambda1, 1.10f);

	float adj = 1.0f;
	for (int i = 0; i < 100; i++) {
		adj = runStftCallback(stft, 2000, 50);
	}
	// 100 callbacks * error 0.1 * 5ms / tau 10s = 0.005
	EXPECT_NEAR(adj, 1.005f, 1e-4);
}
