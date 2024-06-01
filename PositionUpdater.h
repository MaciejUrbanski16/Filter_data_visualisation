#pragma once

#include <cmath>
#include <utility>
#define M_PI 3.14159265358979323846

class PositionUpdater
{
public:
	void updatePosition(const double previousX, const double previuosY, const double xDistance, const double yDistance, const double azimuth)
	{
		const double distance = calculateDistance(xDistance, yDistance);
		currentX = (currentX + distance * cos(azimuth));
		currentY = (currentY + distance);

		totalDistace = totalDistace + yDistance;
	}

	std::pair<double, double> getCurrentPosition() const
	{
		return std::make_pair(currentX, currentY);
	}

	std::pair<double, double> updatePosition(const double distanceInPeriod, const double filteredAzimuth)
	{
		return std::pair<double, double>(distanceInPeriod * sin(filteredAzimuth * M_PI / 180.0), distanceInPeriod * cos(filteredAzimuth * M_PI / 180.0));
	}

	double getTotalDistance()
	{
		return totalDistace;
	}

private:
	double calculateDistance(const double xDistance, const double yDistance)
	{
		const double temp{ pow(xDistance,2) + pow(yDistance,2) };
		return sqrt(temp);
	}

	double currentX{ 0.0 };
	double currentY{ 0.0 };
	double totalDistace{ 0.0 };
};

