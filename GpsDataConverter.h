#pragma once

#include <vector>
#include <string>

class GpsDataConverter
{
public:
	bool handleGpsData(const std::vector<std::string>& measurements, bool withExpectedPosition);
	double getLatitude() const { return latitude; }
	double getLongitude() const { return longitude; }
	double getOrientation() const { return gpsOrientationDegree; }
	double getVelocityKmph() const { return velocityKmph; }
	double getVelocityMperS() const { return velocityKmph * (1000.0 / 3600.0); }
	uint8_t getSatellitesNumber() const { return satellites; }
	double getExpectedPositionX() const { return expectedPositionX; }
	double getExpectedPositionY() const { return expectedPositionY; }

private:
	bool isNumber(const std::string& meas);

	double latitude{ 0.0 };
	double longitude{ 0.0 };
	double gpsOrientationDegree{ 0.0 };
	double velocityKmph{ 0.0 };
	double velocityMperS{ 0.0 };
	uint8_t satellites{ 0 };
	double expectedPositionX{ 0.0 };
	double expectedPositionY{ 0.0 };

	bool isNewGpsData{ false };
};

