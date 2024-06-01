#pragma once

#include "RawMeasurements.h"
#include <Eigen/Dense>

struct TransformedAccel
{
	double xAcc{ 0.0 };
	double yAcc{ 0.0 };
	double zAcc{ 0.0 };
};

struct XyDistance
{
	double xDistance;
	double yDistance;
};

class AccelTransform
{
public:
	TransformedAccel transform(MeasurementsController& rawMeasurement);
	XyDistance getXyDistance(const uint32_t deltaTimeMs);

	TransformedAccel transformQuaternions(const MeasurementsController& rawMeasurement);
private:
	double degreeToRad(double degree) const;

	TransformedAccel transformedAccel{ 0.0,0.0,0.0 };

	Eigen::Matrix3d Rotation;
	double matrix[3][3];
	Eigen::Matrix3d rotationMatrix;

	double x{ 0.0 };
	double y{ 0.0 };
	double z{ 0.0 };
	int counterOfSamples = 0;

	constexpr static uint8_t X_AXIS{ 0u };
	constexpr static uint8_t Y_AXIS{ 1u };
	constexpr static uint8_t Z_AXIS{ 2u };
};

