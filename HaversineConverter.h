#pragma once
#include <cmath>
#include <utility>
#include <optional>
#include "wx/math.h"

struct GPSPoint 
{
	double latitude;
	double longitude;
};

struct GpsDistanceAngular
{
	double xPosition{ 0.0 };
	double yPosition{ 0.0 };
	double velocity{ 0.0 };
	double angle{ 0.0 };
	double expectedXposition{ 0.0 };
	double expectedYposition{ 0.0 };
	double distanceInPeriod{ 0.0 };

	double xPositionBasedOnDistanceAndAzimuth{ 0.0 };
	double yPositionBasedOnDistanceAndAzimuth{ 0.0 };
};

class HaversineConverter
{
public:
	std::optional<GpsDistanceAngular> calculateCurrentPosition(const double longitude, const double latitude)
	{
		if (fabs(longitude) < 10 or fabs(latitude) < 10)
		{
			return std::nullopt;
		}
		if (isFirstMeasurement)
		{
			actualGpsPoint.latitude = latitude;
			actualGpsPoint.longitude = longitude;
			isFirstMeasurement = false;
			return GpsDistanceAngular{ 0.0, 0.0, 0.0 };
		}

		previousGpsPoint = actualGpsPoint;
		actualGpsPoint.latitude = latitude;
		actualGpsPoint.longitude = longitude;

		const double distance = calculateDistance(longitude, latitude);

		angleFromGps = std::atan2(actualGpsPoint.longitude - previousGpsPoint.longitude,
			actualGpsPoint.latitude - previousGpsPoint.latitude);

		gpsDistanceAngular.xPosition = gpsDistanceAngular.xPosition + (distance * cos(angleFromGps));
		gpsDistanceAngular.yPosition = gpsDistanceAngular.yPosition + (distance * sin(angleFromGps));
		gpsDistanceAngular.angle = angleFromGps;

		return gpsDistanceAngular;
	}

private:
	double calculateDistance(const double longitude, const double latitude)
	{
		double deltaLat = toRadians(previousGpsPoint.latitude - actualGpsPoint.latitude);
		double deltaLon = toRadians(previousGpsPoint.longitude - actualGpsPoint.longitude);

		double a = sin(deltaLat / 2.0) * sin(deltaLat / 2.0) +
			cos(toRadians(actualGpsPoint.latitude)) * cos(toRadians(previousGpsPoint.latitude)) *
			sin(deltaLon / 2.0) * sin(deltaLon / 2.0);

		double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

		return earthRadius * c;
	}

	double toRadians(const double degree)
	{
		return degree* (2.0 * M_PI / 360.0);
	}

	GPSPoint previousGpsPoint{ 0.0, 0.0 };
	GPSPoint actualGpsPoint{ 0.0, 0.0 };

	double xPosition{ 0.0 };
	double yPosition{ 0.0 };
	double angleFromGps{ 0.0 };
	GpsDistanceAngular gpsDistanceAngular{ 0.0,0.0,0.0 };

	bool isFirstMeasurement{ true };

	static constexpr double earthRadius = 6371000.0;
};

