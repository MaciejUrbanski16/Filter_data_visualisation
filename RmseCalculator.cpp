#include "RmseCalculator.h"
#include <stdlib.h>
#include <cmath>
#include <limits>

double RmseCalculator::calculateAzimuthRmse(const double expectedAzimuth, const double filteredAzimuth)
{
    numOfAzimuthSamples++;
    currentSumOfAzimuuthDiffs += pow((std::fabs(expectedAzimuth - filteredAzimuth)),2) / numOfAzimuthSamples;
    return sqrt(currentSumOfAzimuuthDiffs);
}

double RmseCalculator::calculatePositionXRmse(const double expectedXposition, const double filteredXposition)
{
    numOfPositionSamples++;
    currentSumOfPositionXDiffs += pow((std::fabs(expectedXposition - filteredXposition)), 2) / numOfPositionSamples;
    return sqrt(currentSumOfPositionXDiffs);
}

double RmseCalculator::calculatePositionYRmse(const double expectedYposition, const double filteredYposition)
{
    //numOfPositionSamples++;
    currentSumOfPositionYDiffs += pow((std::fabs(expectedYposition - filteredYposition)), 2) / numOfPositionSamples;
    return sqrt(currentSumOfPositionYDiffs);
}

double RmseCalculator::calculateDiffBetweenExpectedAndCurrentAzimuth(const double expectedAzimuth, const double filteredAzimuth)
{
    if (expectedAzimuth - filteredAzimuth > 50)
    {
        return 10.0;
    }
    return expectedAzimuth - filteredAzimuth;
}

double RmseCalculator::calculateDiffBetweenExpectedAndCurrentPosition(const ExpectedPosition& expectedPosition, const std::pair<double, double> filteredPosition)
{
    return sqrt(pow(filteredPosition.first - expectedPosition.expectedX, 2) + pow(filteredPosition.second - expectedPosition.expectedY, 2));
}

double RmseCalculator::calculateDiffExpectedVsFiltered(const PlotElementsBuffer& expectedBuffer, const PlotElementsBuffer& filteredBuffer)
{
    if (filteredBuffer.getBuffer().empty())
    {
        return 0.0;
    }
    const auto lastFiltered = filteredBuffer.getBuffer().back();
    const auto sizeOfExpectedBuffer = expectedBuffer.getBuffer().size();

    double distance{ std::numeric_limits<double>::max() };

    double expectedXForRmse{ 0.0 };
    double expectedYForRmse{ 0.0 };

    if (sizeOfExpectedBuffer < 150)
    {
        for (int i = 0; i < sizeOfExpectedBuffer; i++)
        {
            const auto xExpected{ expectedBuffer.getBuffer()[i].x };
            const auto yExpected{ expectedBuffer.getBuffer()[i].y };
            const auto xFiltered{ lastFiltered.x };
            const auto yFiltered{ lastFiltered.y };
            const double tempDistance = calculateDiffBetweenExpectedAndCurrentPosition(
                ExpectedPosition{ xExpected , yExpected },
                std::pair<double, double>{xFiltered, yFiltered});
            if (distance > tempDistance)
            {
                distance = tempDistance;
                expectedXForRmse = xExpected;
                expectedYForRmse = yExpected;
            }
        }
    }
    else
    {
        for (int i = sizeOfExpectedBuffer - 150; i < sizeOfExpectedBuffer; i++)
        {
            const auto xExpected{ expectedBuffer.getBuffer()[i].x };
            const auto yExpected{ expectedBuffer.getBuffer()[i].y };
            const auto xFiltered{ lastFiltered.x };
            const auto yFiltered{ lastFiltered.y };
            const double tempDistance = calculateDiffBetweenExpectedAndCurrentPosition(
                ExpectedPosition{ xExpected , yExpected },
                std::pair<double, double>{xFiltered, yFiltered});
            if (distance > tempDistance)
            {
                distance = tempDistance;
                expectedXForRmse = xExpected;
                expectedYForRmse = yExpected;
            }
        }
    }
    positionXrmse = calculatePositionXRmse(expectedXForRmse, lastFiltered.x);
    positionYrmse = calculatePositionYRmse(expectedYForRmse, lastFiltered.y);
    return distance;
}
