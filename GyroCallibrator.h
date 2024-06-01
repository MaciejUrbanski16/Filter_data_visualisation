#pragma once
#include <cstdint>
#include <limits>
#include <cmath>
#include <tuple>
#include "RawMeasurements.h"

class GyroCallibrator
{
public:
	void collectDataForCallibration(MeasurementsController& rawMeasurement,
		const double rollFromAcc, const double pitchFromAcc, const double yawFromMagn);
private:
	void resetMonitoring();

	uint16_t counterOfSamples{ 0 };
	constexpr static uint16_t numberOfSamplesToMonitor{ 50 };

	constexpr static double thresholdForXCallibration{ 18 };
	constexpr static double thresholdForYCallibration{ 18 };
	constexpr static double thresholdForZCallibration{ 24 };

	bool isGyroXcallibPossible{ false };
	bool isGyroYcallibPossible{ false };
	bool isGyroZcallibPossible{ false };

	double minRoll{ std::numeric_limits<double>::max() };
	double maxRoll{ -std::numeric_limits<double>::max() };
	double minPitch{ std::numeric_limits<double>::max() };
	double maxPitch{ -std::numeric_limits<double>::max() };
	double minYaw{ std::numeric_limits<double>::max() };
	double maxYaw{ -std::numeric_limits<double>::max() };

	double meanXgyroValue{ 0.0 };
	double meanYgyroValue{ 0.0 };
	double meanZgyroValue{ 0.0 };

	double sumXgyro{ 0.0 };
	double sumYgyro{ 0.0 };
	double sumZgyro{ 0.0 };
};

