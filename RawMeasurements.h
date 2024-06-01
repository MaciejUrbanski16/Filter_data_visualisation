#pragma once
#include <vector>
#include <string>
#include <utility>
#include <chrono>
#include <cmath>
#include <math.h>

#include "MagnetometerCallibrator.h"
#include "AppLogger.h"  
#define M_PI 3.1415926

#define EXPECTED_FRAME_SIZE 11
#define REAL_TIME_MEASUREMENT false

struct CompensatedAccData 
{
	double xAcc;
	double yAcc;
};

struct CompensatedVelocityData
{
	double xVelocity;
	double yVelocity;
};

struct CompensatedPositionData
{
	double xPosition;
	double yPosition;
};

class MeasurementsController
{
public:
	MeasurementsController(AppLogger& appLogger, const double newRawGrawity, const double newXBias, const double newYBias,
		const double newXGyroBias, const double newYGyroBias, const double newZGyroBias, 
		MagnetometerCallibrator& magnetometerCallibrator): appLogger(appLogger), magnetometerCallibrator(magnetometerCallibrator)
	{
		setRawGrawity(newRawGrawity);
		setXBias(newXBias);
		setYBias(newYBias);

		setXGyroBias(newXGyroBias);
		setYGyroBias(newYGyroBias);
		setZGyroBias(newZGyroBias);

		this->xMagnOffset = magnetometerCallibrator.getXoffset();
		this->yMagnOffset = magnetometerCallibrator.getYoffset();
	}

	bool assign(const std::vector<std::string>& measurements, const std::vector<std::string>& expectedPosition, const uint32_t deltaTimeMs, const bool isRealTimeMeasurement, const bool isGpsComing)
	{
		if (measurements.size() == EXPECTED_FRAME_SIZE)
		{
			if (isNumber(measurements[0]) && isNumber(measurements[1]) && isNumber(measurements[2]) &&
				isNumber(measurements[3]) && isNumber(measurements[4]) && isNumber(measurements[5]) &&
				isNumber(measurements[6]) && isNumber(measurements[7]))

			{
				xAcc = std::stoi(measurements[0]);
				yAcc = std::stoi(measurements[1]);
				zAcc = std::stoi(measurements[2]);

				xGyro = std::stoi(measurements[3]);
				yGyro = std::stoi(measurements[4]);
				zGyro = std::stoi(measurements[5]);

				xMagn = std::stoi(measurements[6]);
				yMagn = std::stoi(measurements[7]);
				

				if (isRealTimeMeasurement)
				{
					zMagn = std::stoi(measurements[8]);
					xMagn = xMagn - xMagnOffset;
					yMagn = yMagn - yMagnOffset;
					calculateOrientationDegree();
				}
				else
				{
					if (isNumber(measurements[9]))
					{
						expectedOrientation = std::stod(measurements[9]);
					}
					
					try
					{
						orientationDegree = std::stod(measurements[8]);
					}
					catch (const std::invalid_argument& ia) 
					{
						const std::string errMeasConversion{ "ERR when measurement conversion from string to double - some measurement is not double!!" };
						appLogger.logErrMeasurementConversion(errMeasConversion);
					}
					catch (const std::out_of_range& oor) 
					{
						const std::string errMeasConversion{ "ERR when measurement conversion from string to double - some measurement is out of range!!" };
						appLogger.logErrMeasurementConversion(errMeasConversion);
					}
					catch (const std::exception& e) 
					{
						const std::string errMeasConversion{ "ERR when measurement conversion from string to double!!" };
						appLogger.logErrMeasurementConversion(errMeasConversion);
					}
					
				}

				this->deltaTimeMs = deltaTimeMs;

				calculateAccInMPerS2();
				calculateAngleVelocity();

				calculateXYvelocity();
				calculateDistance();

				appLogger.logHandledMeas(xAcc, yAcc, zAcc, xGyro, yGyro, zGyro, xMagn, yMagn,
					xAccMPerS2, yAccMPerS2, zAccMPerS2, xVelocity, yVelocity,
					xDistance, yDistance, orientationDegree, longitude, latitude, deltaTimeMs);

				if (isRealTimeMeasurement)
				{
					const double orientationDegreeWithCallibration = getAzimuth();
					appLogger.logHandledMeasIntoCSV(xAcc, yAcc, zAcc, xGyro, yGyro, zGyro, xMagn, yMagn,
						xAccMPerS2, yAccMPerS2, zAccMPerS2, xVelocity, yVelocity,
						xDistance, yDistance, orientationDegree, orientationDegreeWithCallibration, isGpsComing, deltaTimeMs);
				}

				return true;
			}
			const std::string errMeasConversion{ "ERR when measurement conversion from string to int16_t - some measurement is not integer!!" };
			appLogger.logErrMeasurementConversion(errMeasConversion);
			return false;
		}
		const std::string errMeasConversion{ "ERR when measurement conversion from string to int16_t - incorrect size of measuremets struct!!" };
		appLogger.logErrMeasurementConversion(errMeasConversion);
		return false;
	}

	void correctGyroData(const double xGyroInStableState, const double yGyroInStableState, const double zGyroInStableState)
	{
		correctXGyro = xGyroInStableState;
		correctYGyro = yGyroInStableState;
		correctZGyro = zGyroInStableState;
	}

	double getXaccMPerS2() const { return xAccMPerS2; }
	double getYaccMPerS2() const { return yAccMPerS2; }
	double getZaccMPerS2() const { return zAccMPerS2; }
	CompensatedAccData getCompensatedAccData() const { return compensatedAccData; }
	CompensatedVelocityData getCompensatedVelocityData() const { return compensatedVelocityData; }
	CompensatedPositionData getCompensatedPositionData() const { return compensatedPositionData; }

	double getXangleVelocityDegreePerS() const { return xAngleVelocityDegPerS - correctXGyro; }
	double getYangleVelocityDegreePerS() const { return yAngleVelocityDegPerS - correctYGyro; }
	double getZangleVelocityDegreePerS() const { return zAngleVelocityDegPerS - correctZGyro; }
	double getLongitude() const { return longitude; }
	double getLatitude() const { return latitude; }

	double getXvelocityMperS() const { return xVelocity; }
	double getYvelocityMperS() const { return yVelocity; }
	double getActualVelocityMperSec() const { return actualVelocity; }

	double getXDistance() const { return xDistance; }
	double getYDistance() const { return yDistance; }

	double getExpectedPositionX() const { return expectedPositionX; }
	double getExpectedPositionY() const { return expectedPositionY; }
	double getExpectedOrientation() const { return expectedOrientation; }

	//radians
	double getRollFromAcc() const
	{
		return atan2(static_cast<float>(yAcc + yBias), static_cast<float>(zAcc));// *(180.0 / M_PI);
	}
	double getPitchFromAcc() const
	{
		return atan2((static_cast<float>(-(xAcc + xBias))),
			sqrt(static_cast<float>(yAcc + yBias) * static_cast<float>(yAcc + yBias) + static_cast<float>(zAcc) * static_cast<float>(zAcc)));// *(180.0 / M_PI);
	}
	double getYawFromMagn()
	{
		double roll = getRollFromAcc();
		double pitch = getPitchFromAcc();
		double compensatedX = static_cast<double>(xMagn * cos(pitch)) - static_cast<double>(yMagn * sin(roll) * sin(pitch)) + static_cast<double>(zMagn * cos(roll) * sin(pitch));
		double compensatedY = (static_cast<double>(yMagn * cos(roll)) + static_cast<double>(zMagn * sin(roll)));
		yawCompensated = atan2(compensatedY, compensatedX);
		if (yawCompensated < 0.0)
		{
			yawCompensated += (2.0 * M_PI);
		}
		return yawCompensated;
	}

	//degrees
	double getAzimuth()
	{
		const int16_t magnBiasToNorthInDegrees{ magnetometerCallibrator.getBiasToNorth() };
		double orientationInDegreeWithBias{ (getYawFromMagn()*360.0)/(2*M_PI) + static_cast<double>(magnBiasToNorthInDegrees)};
		if (orientationInDegreeWithBias > 360.0)
		{
			orientationInDegreeWithBias = orientationInDegreeWithBias - 360.0;
		}
		if (orientationInDegreeWithBias < 0)
		{
			orientationInDegreeWithBias = orientationInDegreeWithBias + 360.0;
		}
		magnetometerCallibrator.setCurrentAzimuth(orientationInDegreeWithBias);
		return orientationInDegreeWithBias;
	}
	int16_t getRawXMagn() const { return xMagn; }
	int16_t getRawYMagn() const { return yMagn; }

	int16_t getRawXacc() const { return xAcc; }
	int16_t getRawYacc() const { return yAcc; }
	int16_t getRawZacc() const { return zAcc; }



private:
	void setRawGrawity(const double newRawGrawity) { rawGrawity = newRawGrawity; }
	void setXBias(const double newXBias) { xBias = newXBias; }
	void setYBias(const double newYBias) { yBias = newYBias; }

	void setXGyroBias(const double newXBias) { xAngleVelBias = newXBias; }
	void setYGyroBias(const double newYBias) { yAngleVelBias = newYBias; }
	void setZGyroBias(const double newZBias) { zAngleVelBias = newZBias; }

	void calculateAccInMPerS2()
	{
		xAccMPerS2 = static_cast<double>(xAcc + xBias) * gPhysConst / rawGrawity;
		yAccMPerS2 = static_cast<double>(yAcc + yBias) * gPhysConst / rawGrawity;
		zAccMPerS2 = static_cast<double>(zAcc) * gPhysConst / rawGrawity;

		compensateAccData();
	}

	void compensateAccData()
	{
		double gravityMagnitude = std::sqrt(xAccMPerS2 * xAccMPerS2 +
			yAccMPerS2 * yAccMPerS2 +
			zAccMPerS2 * zAccMPerS2);

		double gravityX = xAccMPerS2 / gravityMagnitude;
		double gravityY = yAccMPerS2 / gravityMagnitude;
		double gravityZ = zAccMPerS2 / gravityMagnitude;

		double compensatedXacc = xAccMPerS2;
		compensatedXacc -= (gravityX * zAccMPerS2);

		double compensatedYacc = yAccMPerS2;
		compensatedYacc -= (gravityY * zAccMPerS2);

		compensatedAccData.xAcc = compensatedXacc;
		compensatedAccData.yAcc = compensatedYacc;
	}

	void calculateAngleVelocity()
	{
		xAngleVelocityDegPerS = static_cast<double>(xGyro + xAngleVelBias) * 250.0f / 32768.0f; //the bias has to be set correctly
		yAngleVelocityDegPerS = static_cast<double>(yGyro + yAngleVelBias) * 250.0f / 32768.0f;
		zAngleVelocityDegPerS = static_cast<double>(zGyro + zAngleVelBias) * 250.0f / 32768.0f;
	}

	void calculateXYvelocity()
	{
		const double timeIntervalSec = static_cast<double>(deltaTimeMs)/1000;
		xVelocity =  xAccMPerS2 * timeIntervalSec;
		yVelocity +=  yAccMPerS2 * timeIntervalSec;
		actualVelocity = actualVelocity + yVelocity;

		previousXvelocity = xVelocity;
		previousYvelocity = yVelocity;

		calculateCompansatedXYvelocity();
	}

	void calculateCompansatedXYvelocity()
	{
		const double timeIntervalSec = static_cast<double>(deltaTimeMs) / 1000;
		compensatedVelocityData.xVelocity = xAccMPerS2 * timeIntervalSec;
		compensatedVelocityData.yVelocity = yAccMPerS2 * timeIntervalSec;
	}

	void calculateDistance()
	{
		const double timeIntervalSec = static_cast<double>(deltaTimeMs) / 1000;
		xDistance = xVelocity * timeIntervalSec;
		yDistance = yVelocity * timeIntervalSec;

		calculateCompansatedDistance();
	}

	void calculateCompansatedDistance()
	{
		const double timeIntervalSec = static_cast<double>(deltaTimeMs) / 100;
		compensatedPositionData.xPosition = xVelocity * timeIntervalSec;
		compensatedPositionData.yPosition = yVelocity * timeIntervalSec;
	}

	void calculateOrientationDegree()
	{
		orientationDegree = atan2(static_cast<double>(yMagn),
			static_cast<double>(xMagn)) * (360.0f /(2.0 * M_PI));
	}

	bool isNumber(const std::string& meas)
	{
		for (const char c : meas)
		{
			if (not (c == '-' or c =='.' or c == '0' or c == '1' or c == '2' or c == '3' or c == '4' or
				c == '5' or c == '6' or c == '7' or c == '8' or c == '9'))
			{
				
				return false;
			}
		}
		return true;
	}

	bool isFloat(const std::string& meas)
	{
		try {
			size_t pos;
			std::stod(meas, &pos);
			return pos == meas.length();
		}
		catch (...) {
			return false;
		}
	}

	std::pair<bool, std::string> isMagn(const std::string& amgnMeas)
	{
		std::string magnStr{};
		for (const char c : amgnMeas)
		{
			if (not (c == '-' or c == '0' or c == '1' or c == '2' or c == '3' or c == '4' or
				c == '5' or c == '6' or c == '7' or c == '8' or c == '9'))
			{
				if (c == '\r' or c == '\n')
				{
					return { true, magnStr };
				}
				return { false,"" };
			}
			magnStr += c;
		}
		return { true, magnStr };
	}

	int16_t xAcc{ 0xFF };
	int16_t yAcc{ 0xFF };
	int16_t zAcc{ 0xFF };
	int16_t xGyro{ 0xFF };
	int16_t yGyro{ 0xFF };
	int16_t zGyro{ 0xFF };
	int16_t xMagn{ 0xFF };
	int16_t yMagn{ 0xFF };
	int16_t zMagn{ 0xFF };

	int16_t xMagnOffset{ 0 };
	int16_t yMagnOffset{ 0 };

	double expectedPositionX{ 0.0 };
	double expectedPositionY{ 0.0 };
	double expectedOrientation{ 0.0 };

	double longitude{ 0.0 };
	double latitude{ 0.0 };

	int16_t magn{ 0xFF };
	double yawCompensated{ 0.0 };

	double xAccMPerS2{ 0.0 };
	double yAccMPerS2{ 0.0 };
	double zAccMPerS2{ 0.0 };
	CompensatedAccData compensatedAccData{ 0.0,0.0 };
	CompensatedVelocityData compensatedVelocityData{ 0.0,0.0 };
	CompensatedPositionData compensatedPositionData{ 0.0,0.0 };

	double xAngleVelocityDegPerS{ 0.0 };
	double yAngleVelocityDegPerS{ 0.0 };
	double zAngleVelocityDegPerS{ 0.0 };

	double orientationDegree{ 0.0 };
	
	double previousXvelocity{ 0.0 };
	double previousYvelocity{ 0.0 };
	double xVelocity{ 0.0 };
	double yVelocity{ 0.0 };
	double actualVelocity{ 0.0 };

	double xDistance{ 0.0 };
	double yDistance{ 0.0 };

	uint32_t deltaTimeMs{ 100 };
	
	static constexpr double gPhysConst = 9.803;
	double rawGrawity = 16000.0;
	double xBias = 80.0;
	double yBias = 40.0;

	double xAngleVelBias{ -18000.0};
	double yAngleVelBias{ 12900.0};
	double zAngleVelBias{ 9200.0 };

	double correctXGyro{ 0.0 };
	double correctYGyro{ 0.0 };
	double correctZGyro{ 0.0 };

	AppLogger& appLogger;
	MagnetometerCallibrator& magnetometerCallibrator;
};