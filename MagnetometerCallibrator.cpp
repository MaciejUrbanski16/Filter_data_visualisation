#include "MagnetometerCallibrator.h"

void MagnetometerCallibrator::collectData(const int16_t xMagn, const int16_t yMagn)
{
	sumOfXMagn += xMagn;
	sumOfYMagn += yMagn;

	numOfSamples++;
}

void MagnetometerCallibrator::callibrate()
{
	xOffset = sumOfXMagn / numOfSamples;
	yOffset = sumOfYMagn / numOfSamples;
}

void MagnetometerCallibrator::setBiasToNorth(const int16_t bias)
{
	biasToNorth = bias;
}

void MagnetometerCallibrator::setCurrentAzimuth(const double azimuth)
{
	currentAzimuth = azimuth;
}
