#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CsvMeasurementReader
{
public:
	bool openFile(const std::string filePath);
	bool openFileWithGpsData(const std::string filePath);
	bool openFileWithExpectedPositionData(const std::string filePath);

	std::vector<std::string> readCSVrow(char delimiter = ',');
	std::vector<std::string> readCSVrowGpsData(char delimiter = ',');
	std::vector<std::string> readCSVExpectedPositionData(char delimiter = ',');

	void setReadMeasurementFromBegining();
	void setReadGpsDataFromBegining();
	~CsvMeasurementReader();

private:
	std::ifstream fileMeasurementsData;
	std::vector<std::string> headers;
	std::ifstream fileGpsData;
	std::vector<std::string> headersGpsData;
	std::ifstream fileExpectedPositionData;
	std::vector<std::string> headersExpectedPositionData;
};

