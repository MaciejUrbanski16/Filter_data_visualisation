#include "SerialComSensorDataReceiver.h"

void SerialComSensorDataReceiver::exctractAccGyroMeasurements(const std::string& frameWithMeasurements)
{
    std::istringstream ss(frameWithMeasurements);
    std::string token;
    int nr = 0;
    while (std::getline(ss, token, measumrementsDelimiter))
    {
        if (nr < 9)
        {
            nr++;
            exctractedMeasurements.push_back(token);
        }
    }
}

void SerialComSensorDataReceiver::exctractMagnMeasurements(const std::string& frameWithMeasurements)
{
    std::istringstream ss(frameWithMeasurements);
    std::string token;
    int nr = 0;
    while (std::getline(ss, token, measumrementsDelimiter))
    {
        if (nr < 3)
        {
            nr++;
            exctractedMeasurements.push_back(token);
        }
    }
}

std::vector<std::string> SerialComSensorDataReceiver::exctractGpsData(const std::string& frameWithMeasurements)
{
    std::vector<std::string> values;
    size_t startPos = 0;
    size_t endPos = frameWithMeasurements.find('_');

    while (endPos != std::string::npos) {
        std::string substr = frameWithMeasurements.substr(startPos, endPos - startPos);
        values.push_back(substr);
        startPos = endPos + 1;
        endPos = frameWithMeasurements.find('_', startPos);
    }
    return values;
}

void SerialComSensorDataReceiver::logIntoFile(const std::string& frameWithMeasurements, const std::vector<std::string>& exctractedMeasurements)
{
    char delimiter = 'X';
    size_t position = frameWithMeasurements.find(delimiter);
    std::string rawRemoteDataString = frameWithMeasurements.substr(0, position);

    std::stringstream ssSensorData;
    const auto currentTime = getCurrentTimeWithMilliSeconds();
    ssSensorData << currentTime << " RAW SENSOR FRAME: ";/* << rawRemoteDataString << " "; /*" ---> The remoteSensorData was received: Xacc:" << XaccAsString << " Yacc:" <<
        YaccAsString << " Zacc:" << ZaccAsString << " Xgyr:" << XgyrAsString << " Ygyr:" << YgyrAsString
        << " Zgyr:" << ZgyrAsString << " magn: " << magnAsString << '\n'*/
    for (const auto& measumrentStr : exctractedMeasurements)
    {
        ssSensorData << measumrentStr << ", ";
    }
    ssSensorData << '\n';
    outputFile << ssSensorData.str();
}

void SerialComSensorDataReceiver::logReadRemoteData(const std::string& frameWithMeasurements)
{
    const auto currentTime = getCurrentTimeWithMilliSeconds();
    std::stringstream ssSensorData;
    ssSensorData << currentTime << " FRAME: " << frameWithMeasurements << "\r\n";
    outputFile << ssSensorData.str();
}

void SerialComSensorDataReceiver::logErrReadRemoteData()
{
    std::stringstream errRead;
    const auto currentTime = getCurrentTimeWithMilliSeconds();
    errRead << currentTime << " The ERR occurred during read of measurement data!!!\n";
    outputFile << errRead.str();
}

std::string SerialComSensorDataReceiver::getCurrentTimeWithMilliSeconds()
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