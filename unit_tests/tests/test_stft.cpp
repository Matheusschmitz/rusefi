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
	static constexpr int MAX_STEPS = 1000;
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

	float trim = 1.0f;
	float minLambda = plant.sensorLambda;
	for (int i = 0; i < FopdtLambdaPlant::MAX_STEPS; i++) { // 5 seconds
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
