#include "GyroCallibrator.h"

void GyroCallibrator::collectDataForCallibration(MeasurementsController& rawMeasurement, const double rollFromAcc, const double pitchFromAcc, const double yawFromMagn)
{

	if (minRoll > rollFromAcc)
	{
		minRoll = rollFromAcc;
	}
	if (maxRoll < rollFromAcc)
	{
		maxRoll = rollFromAcc;
	}
	if (minPitch > pitchFromAcc)
	{
		minPitch = pitchFromAcc;
	}
	if (maxPitch < pitchFromAcc)
	{
		maxPitch = pitchFromAcc;
	}
	if (minYaw > yawFromMagn)
	{
		minYaw = yawFromMagn;
	}
	if (maxYaw < yawFromMagn)
	{
		maxYaw = yawFromMagn;
	}

	sumXgyro = sumXgyro + rawMeasurement.getXangleVelocityDegreePerS();
	sumYgyro = sumYgyro + rawMeasurement.getYangleVelocityDegreePerS();
	sumZgyro = sumZgyro + rawMeasurement.getZangleVelocityDegreePerS();

	counterOfSamples++;

	if (counterOfSamples == numberOfSamplesToMonitor)
	{
		//tollerance 1.5 1.5 4
		const double diffInRoll = maxRoll - minRoll;
		const double diffInPitch = maxPitch - minPitch;
		const double diffInYaw = maxYaw - minYaw;

		if (fabs(diffInRoll) < thresholdForXCallibration)
		{
			isGyroXcallibPossible = true;
			meanXgyroValue = sumXgyro / static_cast<double>(counterOfSamples);
		}
		if (fabs(diffInPitch) < thresholdForYCallibration)
		{
			isGyroYcallibPossible = true;
			meanYgyroValue = sumYgyro / static_cast<double>(counterOfSamples);
		}
		if (fabs(diffInYaw) < thresholdForZCallibration)
		{
			isGyroZcallibPossible = true;
			meanZgyroValue = sumZgyro / static_cast<double>(counterOfSamples);
		}
		resetMonitoring();
	}

	rawMeasurement.correctGyroData(meanXgyroValue, meanYgyroValue, meanZgyroValue);
}

void GyroCallibrator::resetMonitoring()
{
	minRoll = std::numeric_limits<double>::max();
	maxRoll = -std::numeric_limits<double>::max();
	minPitch = std::numeric_limits<double>::max();
	maxPitch = -std::numeric_limits<double>::max();
	minYaw = std::numeric_limits<double>::max();
	maxYaw = -std::numeric_limits<double>::max();

	isGyroXcallibPossible = false;
	isGyroYcallibPossible = false;
	isGyroZcallibPossible = false;

	sumXgyro = 0.0;
	sumYgyro = 0.0;
	sumZgyro = 0.0;

	counterOfSamples = 0;
}
