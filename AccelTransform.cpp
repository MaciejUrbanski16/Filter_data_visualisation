#include "AccelTransform.h"

TransformedAccel AccelTransform::transform(MeasurementsController& rawMeasurement)
{
    const double xAccMPerS2 = rawMeasurement.getXaccMPerS2();
    const double yAccMPerS2 = rawMeasurement.getYaccMPerS2();
    const double zAccMPerS2 = rawMeasurement.getZaccMPerS2();

    const double roll = rawMeasurement.getRollFromAcc();
    const double pitch = rawMeasurement.getPitchFromAcc();
    const double yaw =  rawMeasurement.getAzimuth() * M_PI / 180.0;

    Eigen::Vector3d accelAfterLocalCoordTransformed(0.0,0.0,0.0);
    Eigen::Vector3d acceleration(xAccMPerS2, yAccMPerS2, zAccMPerS2);

    if (counterOfSamples % 4 == 0)
    {
        const double r = sqrtf(xAccMPerS2 * xAccMPerS2 + yAccMPerS2 * yAccMPerS2 + zAccMPerS2 * zAccMPerS2);
        const double x = xAccMPerS2 / r;
        const double y = yAccMPerS2 / r;
        const double z = zAccMPerS2 / r;

        const double x2 = x * x;
        const double y2 = y * y;

        matrix[X_AXIS][X_AXIS] = (y2 - (x2 * z)) / (x2 + y2);
        matrix[X_AXIS][Y_AXIS] = ((-x * y) - (x * y * z)) / (x2 + y2);
        matrix[X_AXIS][Z_AXIS] = x;
        matrix[Y_AXIS][X_AXIS] = ((-x * y) - (x * y * z)) / (x2 + y2);
        matrix[Y_AXIS][Y_AXIS] = (x2 - (y2 * z)) / (x2 + y2);
        matrix[Y_AXIS][Z_AXIS] = y;
        matrix[Z_AXIS][X_AXIS] = -x;
        matrix[Z_AXIS][Y_AXIS] = -y;
        matrix[Z_AXIS][Z_AXIS] = -z;


        Eigen::Matrix3d Rz;
        Rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;

        // Obliczanie k¹ta wokó³ osi Y (pitch)
        Eigen::Matrix3d Ry;
        Ry << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);

        // Obliczanie k¹ta wokó³ osi X (roll)
        Eigen::Matrix3d Rx;
        Rx << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);

        rotationMatrix = Rz * Ry * Rx;

    }
  
    const double xAccTransformed =
        matrix[X_AXIS][X_AXIS] * xAccMPerS2
        + matrix[X_AXIS][Y_AXIS] * yAccMPerS2
        + matrix[X_AXIS][Z_AXIS] * zAccMPerS2;

    const double yAccTransformed =
        matrix[Y_AXIS][X_AXIS] * xAccMPerS2
        + matrix[Y_AXIS][Y_AXIS] * yAccMPerS2
        + matrix[Y_AXIS][Z_AXIS] * zAccMPerS2;

    const double zAccTransformed =
        matrix[Z_AXIS][X_AXIS] * xAccMPerS2
        + matrix[Z_AXIS][Y_AXIS] * yAccMPerS2
        + matrix[Z_AXIS][Z_AXIS] * zAccMPerS2;

    Eigen::Matrix3d transposedRotationMatrix = rotationMatrix.transpose();
    Eigen::Vector3d localAcceleration = transposedRotationMatrix * acceleration;
    accelAfterLocalCoordTransformed = rotationMatrix.transpose() * acceleration;

    //accelAfterLocalCoordTransformed[0] = xAccTransformed;
    //accelAfterLocalCoordTransformed[1] = yAccTransformed;
    //accelAfterLocalCoordTransformed[2] = zAccTransformed;
    
    counterOfSamples++;

    transformedAccel = TransformedAccel{ accelAfterLocalCoordTransformed[0],accelAfterLocalCoordTransformed[1],accelAfterLocalCoordTransformed[2] };
    return transformedAccel;
}

XyDistance AccelTransform::getXyDistance(const uint32_t deltaTimeMs)
{
    const double timeIntervalSec = static_cast<double>(deltaTimeMs) / 1000.0;
    const double xVelocity = transformedAccel.xAcc * timeIntervalSec;
    const double yVelocity = transformedAccel.yAcc * timeIntervalSec;
    const double xDistance = xVelocity * timeIntervalSec;
    const double yDistance = yVelocity * timeIntervalSec;

    return XyDistance{ xDistance, yDistance };
}

double AccelTransform::degreeToRad(double degrees) const
{
    return degrees * 2.0 * M_PI / 360.0;
}