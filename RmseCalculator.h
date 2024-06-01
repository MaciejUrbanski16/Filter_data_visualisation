#pragma once
#include <cstdint>
#include <utility>
#include "ExpectedPosition.h"
#include "PlotElementsBuffer.h"

class RmseCalculator
{
public:
	double calculateAzimuthRmse(const double expectedAzimuth, const double filteredAzimuth);
	double calculatePositionXRmse(const double expectedXposition, const double filteredXpzimuth);
	double calculatePositionYRmse(const double expectedYposition, const double filteredYposition);
	double calculateDiffBetweenExpectedAndCurrentAzimuth(const double expectedAzimuth, const double filteredAzimuth);
	double calculateDiffBetweenExpectedAndCurrentPosition(const ExpectedPosition& expectedPosition, const std::pair<double, double> filteredPosition);

	double calculateDiffExpectedVsFiltered(const PlotElementsBuffer& expectedBuffer, const PlotElementsBuffer& filteredBuffer);

	double positionXrmse{ 0.0 };
	double positionYrmse{ 0.0 };
private:
	uint32_t numOfAzimuthSamples{ 0 };
	uint32_t numOfPositionSamples{ 0 };
	double currentSumOfAzimuuthDiffs{ 0.0 };
	double currentSumOfPositionXDiffs{ 0.0 };
	double currentSumOfPositionYDiffs{ 0.0 };
};

