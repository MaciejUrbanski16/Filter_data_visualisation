#include "KalmanFilters.h"

bool KalmanFilters::makePositionFiltration(
    const MeasurementsController& rawMeasurement, const double currentXPosition, const double currentYPosition,
    const std::optional<GpsDistanceAngular> gpsBasedPosition,
    const TransformedAccel& transformedAccel,
    const double filteredAzimuth,
    const double Xacc, const double Yacc,
    uint32_t deltaTimeUint)
{
    bool doIncrement{ false };
    kf::Matrix<DIM_X, DIM_X> A;

    double deltaTimeMs = static_cast<double>(deltaTimeUint) / 1000.0;
    A <<
        1.0F, 0.0F, deltaTimeMs, 0.0F, 0.5F * deltaTimeMs * deltaTimeMs, 0.0F,
        0.0F, 1.0F, 0.0F, deltaTimeMs, 0.0F, 0.5F * deltaTimeMs * deltaTimeMs,
        0.0F, 0.0F, 1.0F, 0.0F, deltaTimeMs, 0.0F,
        0.0F, 0.0F, 0.0F, 1.0F, 0.0F, deltaTimeMs,
        0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;


    //const auto matRFromGui{ kalmanFilterSetupGui.getMatRacc() };
    //const auto matQFromGui{ kalmanFilterSetupGui.getMatQacc() };
    kf::Matrix<DIM_Z, DIM_Z> matRAccPedestrian;
    kf::Matrix<DIM_X, DIM_X> matQAccPedestrian;

    const auto matR{ kalmanFilterSetupGui.getMatRacc() };
    const auto matQ{ kalmanFilterSetupGui.getMatQacc() };

    kalmanFilterForPosition.predictLKF(A, matQ.value());

    kf::Matrix<DIM_Z, DIM_X> matH;
    kf::Vector<DIM_Z> vecZ;

    if (gpsBasedPosition)
    {
        const double xPosition{ gpsBasedPosition.value().xPosition};
        const double yPosition{ gpsBasedPosition.value().yPosition};
        const double acc{ rawMeasurement.getYaccMPerS2() };
        const double xAcc{ acc * cos(filteredAzimuth * M_PI / 180.0) };
        const double yAcc{ acc * sin(filteredAzimuth * M_PI / 180.0) };

        const double xPositionBasedOnAzimuth{ gpsBasedPosition.value().xPositionBasedOnDistanceAndAzimuth };
        const double yPositionBasedOnAzimuth{ gpsBasedPosition.value().yPositionBasedOnDistanceAndAzimuth };

        matH <<
            1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;

        vecZ << xAcc, yAcc, 0.0, 0.0, xPositionBasedOnAzimuth, yPositionBasedOnAzimuth;
    }
    else
    {
        doIncrement = true;

        const double acc{ rawMeasurement.getYaccMPerS2()};
        const double xAcc{ acc * cos(filteredAzimuth) };
        const double yAcc{ acc * sin(filteredAzimuth) };

        matH << 
            1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F;
        vecZ << xAcc, yAcc, 0.0F, 0.0F, 0.0F, 0.0F;
    }

    kalmanFilterForPosition.setMatH(matH);
    kalmanFilterForPosition.correctLKF(vecZ, matR.value());
    return doIncrement;
}

void KalmanFilters::makeAzimuthFitration(const std::optional<GpsDistanceAngular> gpsBasedPosition,
                                         const double zAngleVelocityDegPerSec, const double azimuthFromMagn, const uint32_t deltaTime)
{
    //X = [yaw,
    //     dYaw]       

    double deltaTimeSec = static_cast<double>(deltaTime) / 1000.0;

    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> A;
    A << 1.0F, deltaTimeSec,
         0.0F, 1.0F;


    //const auto matRAzzFromGui{ kalmanFilterSetupGui.getMatRazimuth() };
    //const auto matQAzzFromGui{ kalmanFilterSetupGui.getMatQazimuth() };
    kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth> matRAzimuthPedestrian;
    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> matQAzimuthPedestrian;
    matRAzimuthPedestrian << 
        10.0F, 0, 0,
        0, 0.001F, 0,
        0, 0, 0.01F;

    double process_variance = 0.00002F;
    double coefficient = 5.0F;
    matQAzimuthPedestrian << 
        coefficient, 0.0F,
        0.0F, coefficient;

    matQAzimuthPedestrian *= process_variance;

    kalmanFilterForAzimuth.setInitialStateForAzimuth(azimuthFromMagn);
    kalmanFilterForAzimuth.predictLKF(A, matQAzimuthPedestrian);
    kf::Matrix<DIM_Z_azimuth, DIM_X_azimuth> matH;


    kf::Vector<DIM_Z_azimuth> vecZ;
    if (gpsBasedPosition)
    {
        //when gps angle available
        vecZ << zAngleVelocityDegPerSec, azimuthFromMagn, gpsBasedPosition.value().angle;
        matH <<
            0, 1,
            1, 0,
            0, 0;
    }
    else
    {
        vecZ << zAngleVelocityDegPerSec, azimuthFromMagn, 0.0F;
        matH <<
            0, 1,
            1, 0,
            0, 0;
    }

    kalmanFilterForAzimuth.setMatH(matH);
    kalmanFilterForAzimuth.correctLKF(vecZ, matRAzimuthPedestrian);
}
