#include <cmath>
#include <iostream>
#include "RelativePositionCalculator.h"

#include "kalman_filter/types.h"
#include "kalman_filter/kalman_filter.h"

void RelativePositionCalculator::calculateActualRelativePosition(const float velocity, const int timeIntervalMs,
    const float azimutDegree) {
    //kf::KalmanFilter<2, 2> kf;
    //y = tg(alpha) * x
    //x^2 + y^2 = r^2
    //x^2 + tg^2(alpha) * x - r^2 = 0
    //a=1   b=tg^2(alpha)     c=r^2
    calculatedRelativePosition.first = previousRelativePosition.first;
    calculatedRelativePosition.second = previousRelativePosition.second;
    const auto lengthOfRoadFromLastMeasurementTimeInterval = velocity * timeIntervalMs;
    const auto radius = pow(lengthOfRoadFromLastMeasurementTimeInterval, 2);
    std::cout << "DEB before calculation(x: " << calculatedRelativePosition.first <<
        "y: " << calculatedRelativePosition.second << ") lengthOfRoad: " << lengthOfRoadFromLastMeasurementTimeInterval
        << " radius: " << radius << "velocity: " << velocity << "timeInterval: " << timeIntervalMs << std::endl;
    std::optional<QuadraticEquationCoefficients> quadraticEquationCoefficients = calculateEquationCoefficients(radius,
        azimutDegree);
    if (quadraticEquationCoefficients) {
        std::optional<std::pair<double, double>> relativePositionToReceivedPosition =
            calculateRelativePositionToActualPosition(quadraticEquationCoefficients.value(), azimutDegree);
        setNewCalculatedRelativePosition(relativePositionToReceivedPosition);
        std::cout << "DEB after calculation (x: " << calculatedRelativePosition.first << " y: " <<
            calculatedRelativePosition.second << ") equation coefficiets: (a: " <<
            quadraticEquationCoefficients.value().a << " b: " << quadraticEquationCoefficients.value().b
            << " c: " << quadraticEquationCoefficients.value().c << " x1: " << quadraticEquationCoefficients.value().x1
            << " x2: " << quadraticEquationCoefficients.value().x2 << " y1: " << quadraticEquationCoefficients.value().y1
            << " y2: " << quadraticEquationCoefficients.value().y2;
    }
}

std::pair<double, double> RelativePositionCalculator::getCalculatedRelativePosition() const {
    return calculatedRelativePosition;
}

void RelativePositionCalculator::setPreviousRelativePosition(std::pair<double, double> previousRelativePosition) {
    this->previousRelativePosition = previousRelativePosition;
}

std::optional<QuadraticEquationCoefficients>
RelativePositionCalculator::calculateEquationCoefficients(const double radius, const float azimutDegree) {
    const std::optional<std::pair<double, double>> aCoefficient = calculateAcoefficient(azimutDegree);
    const double bCoefficient = 0.0;// calculateAcoefficient(azimutDegree);
    //std::tan(azimutDegree)
    if (aCoefficient == std::nullopt) {
        return QuadraticEquationCoefficients{};
    }
    QuadraticEquationCoefficients quadraticEquationCoefficients{ aCoefficient.value().second, bCoefficient, radius * (-1.0) };
    const double delta = pow(quadraticEquationCoefficients.b, 2) - 4 * quadraticEquationCoefficients.a * quadraticEquationCoefficients.c;
    std::cout << "DEB delta: " << delta << "b: " << quadraticEquationCoefficients.b << "4*a*c: " << 4 * static_cast<int>(quadraticEquationCoefficients.a) *
        static_cast<int>(quadraticEquationCoefficients.c) << std::endl;
    quadraticEquationCoefficients.x1 = (-quadraticEquationCoefficients.b - sqrt(delta)) / (2 * quadraticEquationCoefficients.a);
    quadraticEquationCoefficients.x2 = (-quadraticEquationCoefficients.b + sqrt(delta)) / (2 * quadraticEquationCoefficients.a);
    quadraticEquationCoefficients.y1 = (aCoefficient.value().first) * quadraticEquationCoefficients.x1;
    quadraticEquationCoefficients.y2 = (aCoefficient.value().first) * quadraticEquationCoefficients.x2;
    return quadraticEquationCoefficients;
}

std::optional<std::pair<double, double>>
RelativePositionCalculator::calculateRelativePositionToActualPosition(const QuadraticEquationCoefficients& coefficients,
    const float azimut) {
    std::pair<double, double> relativePosition{ 0.0, 0.0 };
    if (azimut > 0.0 && azimut <= 90.0) {
        coefficients.x1 >= 0.0 ? relativePosition.first = coefficients.x1 :
            relativePosition.first = coefficients.x2;
        coefficients.y1 >= 0.0 ? relativePosition.second = coefficients.y1 :
            relativePosition.second = coefficients.y2;
        return std::optional<std::pair<double, double>>(relativePosition);
    }
    else if (azimut > 90.0 && azimut <= 180.0) {
        coefficients.x1 >= 0.0 ? relativePosition.first = coefficients.x1 :
            relativePosition.first = coefficients.x2;
        coefficients.y1 <= 0.0 ? relativePosition.second = coefficients.y1 :
            relativePosition.second = coefficients.y2;
        return std::optional<std::pair<double, double>>(relativePosition);
    }
    else if (azimut > 180.0 && azimut <= 270.0) {
        coefficients.x1 <= 0.0 ? relativePosition.first = coefficients.x1 :
            relativePosition.first = coefficients.x2;
        coefficients.y1 <= 0.0 ? relativePosition.second = coefficients.y1 :
            relativePosition.second = coefficients.y2;
        return std::optional<std::pair<double, double>>(relativePosition);
    }
    else if (azimut > 270.0 && azimut <= 360.0) {
        coefficients.x1 <= 0.0 ? relativePosition.first = coefficients.x1 :
            relativePosition.first = coefficients.x2;
        coefficients.y1 >= 0.0 ? relativePosition.second = coefficients.y1 :
            relativePosition.second = coefficients.y2;
        return std::optional<std::pair<double, double>>(relativePosition);
    }
    else {
        std::cerr << "Azimut " << azimut << "out of allowed range!" << std::endl;
        return std::nullopt;
    }
}

void RelativePositionCalculator::setNewCalculatedRelativePosition(
    const std::optional<std::pair<double, double>> relativePosition) {

    if (relativePosition) {
        calculatedRelativePosition.first += relativePosition.value().first;
        calculatedRelativePosition.second += relativePosition.value().second;
    }
    else {
        std::cerr << "Calculation of new relative position is not successful! nullopt returned" << std::endl;
    }
}

std::optional<std::pair<double, double>> RelativePositionCalculator::calculateAcoefficient(const float degree) {
    if ((degree >= 0.0 and degree < 90.0) or (degree >= 180.0 and degree < 270.0)) {
        // I or III
        std::cout << "1 or 3 Degree: " << degree << "a=tg alpha: " << 90.0 - degree << "      " << std::tan(((90.0 - degree) * (M_PI / 180.0))) << std::endl;
        const double tangensAlfaAsSlopeOfLine = std::tan(((90.0 - degree) * (M_PI / 180.0)));
        const double aCoefficient = pow(std::tan(((90.0 - degree) * (M_PI / 180.0))), 2) + 1.0;
        return std::make_pair(tangensAlfaAsSlopeOfLine, aCoefficient);
    }
    else if (degree >= 90.0 and degree < 180.0) {
        // IV
        std::cout << "4 Degree: " << degree << "a=tg alpha: " << (-(degree - 90.0)) << "         " << std::tan(-(degree - 90.0) * (M_PI / 180.0)) << std::endl;
        const double tangensAlfaAsSlopeOfLine = std::tan((-(degree - 90.0)) * (M_PI / 180.0));
        const double aCoefficient = pow(std::tan((-(degree - 90.0)) * (M_PI / 180.0)), 2) + 1.0;
        return std::make_pair(tangensAlfaAsSlopeOfLine, aCoefficient);
    }
    else if (degree >= 270.0 and degree < 360.0) {
        // II
        std::cout << "2 Degree: " << degree << "a=tg alpha: " << 180.0 + 90.0 - (360.0 - degree) << "          " << std::tan(-(180.0 + 90.0 - (360.0 - degree)) * (M_PI / 180.0)) << std::endl;
        const double tangensAlfaAsSlopeOfLine = std::tan(-(270.0 - (360.0 - degree)) * (M_PI / 180.0));
        const double aCoefficient = pow(std::tan(-(270.0 - (360.0 - degree)) * (M_PI / 180.0)), 2) + 1.0;
        return std::make_pair(tangensAlfaAsSlopeOfLine, aCoefficient);
    }
    else {
        std::cerr << "Invalid azimut" << degree << " during calculation of a coefficient -> return nullopt" << std::endl;
        return std::nullopt;
    }
}
