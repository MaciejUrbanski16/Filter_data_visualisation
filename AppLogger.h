#pragma once
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <vector>

#include "kalman_filter/kalman_filter.h"

class AppLogger
{
public:
	AppLogger() 
	{
		outputFile.open(filePath, std::ios::app);
		kalmanOutputFile.open(kalmanFilePath, std::ios::app);
	}

	~AppLogger()
	{
		if (outputFile.is_open())
		{
			outputFile.close();
		}
		if (kalmanOutputFile.is_open())
		{
			kalmanOutputFile.close();
		}
		if (measurementsCsv.is_open())
		{
			measurementsCsv.close();
		}
		if (gpsCsv.is_open())
		{
			gpsCsv.close();
		}
		if (filteredCsv.is_open())
		{
			filteredCsv.close();
		}
	}

	void logFilteredData(const double filteredAzimuth, const double filteredPositionX, const double filteredPositionY)
	{
		if (!filteredCsv.is_open())
		{
			auto timeAsString = filteredCsvPath;// +getCurrentTimeWithSeconds() + ".txt";
			timeAsString += getCurrentTimeWithSeconds();
			timeAsString += ".txt";
			filteredCsv.open(timeAsString, std::ios::app);

			std::stringstream ssToCsv;
			ssToCsv << "Current time" << ',' << "Filtered azimuth" << ',' 
				<< "Filtered position X" << ',' << "Filtered position Y" << '\n';

			filteredCsv << ssToCsv.str();
		}

		std::stringstream ssToCsv;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ssToCsv << currentTime << ',' << filteredAzimuth << ',' 
			<< filteredPositionX << ',' << filteredPositionY << '\n';

		filteredCsv << ssToCsv.str();
	}

	void logSerialCommStartThread(const std::string& msg)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << " " << msg << '\n';
		outputFile << ss.str();
	}
	void logReceivedDataOnMainThread(const std::vector<std::string>& measurements, const std::string type="")
	{
		std::stringstream ss;
		
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << " " << type << " DATA ON MAIN THREAD: ";
		
		for (const auto& measurement : measurements)
		{
			ss << measurement << " ";
			
		}
		ss << '\n';
		outputFile << ss.str();		
	}
	void logGpsCsvData(const std::vector<std::string>& measurements)
	{
		if (!gpsCsv.is_open())
		{
			auto gpsPathName = gpsCsvPath;
			gpsPathName += getCurrentTimeWithSeconds();
			gpsPathName += ".txt";
			gpsCsv.open(gpsPathName, std::ios::app);

			std::stringstream ssToGpsCsv;
			ssToGpsCsv << "Current time" << ',' << "Latitude" << ',' << "Longitude" << ',' << "Orientation" << ',' << "Velocity" << ','
				<< "Satellites" << '\n';
			gpsCsv << ssToGpsCsv.str();
		}

		std::stringstream ssToCsv;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ssToCsv << currentTime;

		for (const auto& measurement : measurements)
		{
			ssToCsv << ',' << measurement;
		}
		ssToCsv << '\n';
		gpsCsv << ssToCsv.str();
	}
	void logErrThreadDataHandling(const std::string& msg)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << " " << msg << '\n';
		outputFile << ss.str();
	}

	void logHandledMeas(const int16_t xAcc, const int16_t yAcc,
		const int16_t zAcc, const int16_t xGyro, const int16_t yGyro, const int16_t zGyro,
		const int16_t xMagn, const int16_t yMagn, const double xAccMPerS2, const double yAccMPerS2, const double zAccMPerS2,
		const double xVelocity, const double yVelocity, const double xDistance, const double yDistance, const double orientationDegree,
		const double longitude, const double latitude, const uint32_t deltaTimeMs)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << " HANDLED MEASUREMENT DATA: " << "xAcc:" << xAcc << " yAcc:" << yAcc << " zAcc:"
			<< zAcc << " xGyro:" << xGyro << " yGyro:" << yGyro << " zGyro:" << zGyro << " xMagn:" << xMagn << " yMagn:"<< yMagn <<" || " << "xAcc: " << xAccMPerS2
			<< "[m/s2]" << " yAcc: " << yAccMPerS2 << "[m/s2]" << " zAcc: " << zAccMPerS2 << "[m/s2] || xVelocity: " << xVelocity << "[m/s]"
			<< " yVelocity: " << yVelocity << "[m/s] || xDistance: " << xDistance << "[m] yDistance: " << yDistance << "[m] || orientationDegree: " 
			<< orientationDegree << " [deg] || longitude: " << longitude << ", latitude: " << latitude << " Delta time : " << deltaTimeMs<< "[ms]" << '\n';
		outputFile << ss.str();
	}

	void logHandledMeasIntoCSV(const int16_t xAcc, const int16_t yAcc,
		const int16_t zAcc, const int16_t xGyro, const int16_t yGyro, const int16_t zGyro,
		const int16_t xMagn, const int16_t yMagn, const double xAccMPerS2, const double yAccMPerS2, const double zAccMPerS2,
		const double xVelocity, const double yVelocity, const double xDistance, const double yDistance, const double orientationDegree,
		const double orientationDegreeWithCallibration, const bool isGpsComing, const uint32_t deltaTimeMs)
	{
		if (!measurementsCsv.is_open())
		{
			auto timeAsString = measurementsCsvPath;// +getCurrentTimeWithSeconds() + ".txt";
			timeAsString += getCurrentTimeWithSeconds();
			timeAsString += ".txt";
			measurementsCsv.open(timeAsString, std::ios::app);

			std::stringstream ssToCsv;
			ssToCsv << "Current time" << ',' << "X acceleration" << ',' << "Y acceleration" << ',' << "Z acceleration" << ','
				<< "X angle velocity" << ',' << "Y angle velocity" << ',' << "Z angle velocity" << ','
				<< "X raw magnetometer" << ',' << "Y raw magnetometer" << ','
				<< "X accel m/s2" << ',' << "Y accel m/s2" << ',' << "Z accel m/s2" << ','
				<< "X velocity" << ',' << "Y velocity" << ',' << "X distance" << ',' << "Y distance" << ','
				<< "Orientation degree" << ',' << "Orientation degree with callibration" << ',' << "Gps available" << ',' << "Delta time ms" << '\n';
			measurementsCsv << ssToCsv.str();
		}

		std::stringstream ssToCsv;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ssToCsv << currentTime << ',' << xAcc << ',' << yAcc << ',' << zAcc << ','
			<< xGyro << ',' << yGyro << ',' << zGyro << ',' << xMagn << ',' << yMagn << ','
			<< xAccMPerS2 << ',' << yAccMPerS2 << ',' << zAccMPerS2 << ','
			<< xVelocity << ',' << yVelocity << ',' << xDistance << ',' << yDistance << ','
			<< orientationDegree << ',' << orientationDegreeWithCallibration << ',' << isGpsComing << ',' << deltaTimeMs << '\n';
		measurementsCsv << ssToCsv.str();
	}

	void logErrMeasurementConversion(const std::string& msg)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << " " << msg << '\n';
		outputFile << ss.str();
	}

	template <size_t DIM_X, size_t DIM_Z>
	void logKalmanFilterPredictionStep(const kf::KalmanFilter<DIM_X, DIM_Z>& kalmanFilter)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << "\nKALMAN FILTER: Predicted state vector = \n" << kalmanFilter.vecX() << "\n Predicted state covariance = \n" << kalmanFilter.matP() << "  END\n";
		kalmanOutputFile << ss.str();
	}

	template <size_t DIM_X, size_t DIM_Z>
	void logKalmanFilterCorrectionStep(const kf::KalmanFilter<DIM_X, DIM_Z>& kalmanFilter)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << "\nKALMAN FILTER: Corrected state vector = \n" << kalmanFilter.vecX() << "\nCorrected state covariance = \n" << kalmanFilter.matP() << "  END\n";
		kalmanOutputFile << ss.str();
	}

	void logReceivedDataOnTcpPort(const std::string& msg)
	{
		std::stringstream ss;
		auto currentTime = getCurrentTimeWithMilliSeconds();
		ss << currentTime << " " << msg << '\n';
		outputFile << ss.str();
	}
private:
	std::string getCurrentTimeWithMilliSeconds()
	{
		auto currentTime = std::chrono::system_clock::now();
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count();

		constexpr uint16_t msToSec{ 1000 };
		constexpr uint16_t usToMs{ 1000 };
		auto sec = ms / msToSec;
		auto us = (ms % usToMs) * msToSec;

		std::time_t now = std::chrono::system_clock::to_time_t(currentTime);
		std::tm tm_now = *std::localtime(&now);

		std::stringstream timeString;
		timeString << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
		timeString << '.' << std::setfill('0') << std::setw(3) << us;

		std::string currentDateTimeWithMillis = timeString.str();
		return currentDateTimeWithMillis;
	}

	std::string getCurrentTimeWithSeconds()
	{
		auto currentTime = std::chrono::system_clock::now();
		auto seconds = std::chrono::duration_cast<std::chrono::seconds>(currentTime.time_since_epoch()).count();


		std::time_t now = std::chrono::system_clock::to_time_t(currentTime);
		std::tm tm_now = *std::localtime(&now);

		std::stringstream timeString;
		timeString << std::put_time(&tm_now, "%Y-%m-%d_%H-%M-%S");
		

		std::string currentDateTimeWithMillis = timeString.str();
		return currentDateTimeWithMillis;
	}


	std::ofstream outputFile;
	std::string filePath = "appLogs.txt";

	std::ofstream kalmanOutputFile;
	std::string kalmanFilePath = "kalmanFilterLogs.txt";

	std::ofstream measurementsCsv;
	std::string measurementsCsvPath = "measurementsCsv_";

	std::ofstream gpsCsv;
	std::string gpsCsvPath = "gpsCsv_";

	std::ofstream filteredCsv;
	std::string filteredCsvPath = "filtered_";
};
//std::string AppLogger::path = "appLogs.txt";
//std::ofstream AppLogger::outputFile;// = path;

