#pragma once

#include <thread>
#include <atomic>
#include <iostream>
#include <vector>
#include <codecvt>
#include <locale>
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <chrono>

#include <wx/wx.h>
#include <wx/thread.h>

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>

#include "MeasurementCustomizator.h"
#include "GpsDataCustomizator.h"

//wxDECLARE_EVENT(wxEVT_MY_THREAD_EVENT, wxThreadEvent);

wxDEFINE_EVENT(wxEVT_MY_THREAD_EVENT_2, wxThreadEvent);

enum class MeasurementsHandling
{
    WAIT_FOR_ACC_GYRO = 0,
    WAIT_FOR_MAGN,
    READY_TO_SEND
};

class SerialComSensorDataReceiver
{
public:
    SerialComSensorDataReceiver(boost::asio::io_context& io_context, std::string& serial_port_name, wxEvtHandler* parent) :serialPort(io_context, serial_port_name), m_parent(parent)
    {
        serialPort.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serialPort.set_option(boost::asio::serial_port_base::character_size(8));
        serialPort.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serialPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));

        outputFile.open(filePath, std::ios::app);

        start_receive();
    }

    ~SerialComSensorDataReceiver()
    {
        if (outputFile.is_open())
        {
            outputFile.close();
        }
        if (serialPort.is_open())
        {
            serialPort.close();
        }
    }

    void sendData() // to do in case of need bidirectional communication
    {
        std::string data_to_send = "ZZZ";
        try
        {
            boost::asio::write(serialPort, boost::asio::buffer(data_to_send));
        }
        catch (const boost::system::system_error& e) {
            std::cerr << "Error sending data: " << e.what() << std::endl;
        }
    }

private:
    void start_receive()
    {
        serialPort.async_read_some(boost::asio::buffer(remoteSensorData),
            boost::bind(&SerialComSensorDataReceiver::handle_receive,
                this, boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }

    void handle_receive(const boost::system::error_code& error,
        std::size_t bytes_transferred)
    {
        if (!error)
        {
            //DATA STRUCTURE Xacc 6 elements
            //               Yacc 6 elements
            //               Zacc 6 elements
            //               Xgyr 6 elements
            //               Ygyr 6 elements 
            //               Zgyr 6 elements 
            //               Xmagn 6 elements
            //               Ymagn 6 elements
            //               Zmagn 6 elements
            if (measurementsHandling == MeasurementsHandling::WAIT_FOR_ACC_GYRO)
            {
                std::string remoteDataAsString(reinterpret_cast<char*>(remoteSensorData), sizeof(remoteSensorData));
                logReadRemoteData(remoteDataAsString);
                exctractAccGyroMeasurements(remoteDataAsString);
                measurementsHandling = MeasurementsHandling::READY_TO_SEND;
            }

            if (measurementsHandling == MeasurementsHandling::READY_TO_SEND)
            {
                MeasurementCustomizator* event = new MeasurementCustomizator(wxEVT_MY_THREAD_EVENT_2);
                event->SetStringVector(exctractedMeasurements);
                wxQueueEvent(m_parent, event);
                measurementsHandling = MeasurementsHandling::WAIT_FOR_ACC_GYRO;
                exctractedMeasurements.clear();
            }
        }
        else
        {
            logErrReadRemoteData();
            std::cout << "Data not received correctly" << std::endl;
            OutputDebugStringA("Com port data received with Error!");
        }
        start_receive();
    }

    void exctractAccGyroMeasurements(const std::string& frameWithMeasurements);
    void exctractMagnMeasurements(const std::string& frameWithMeasurements);
    std::vector<std::string> exctractGpsData(const std::string& frameWithMeasurements);
    void logIntoFile(const std::string& frameWithMeasurements, const std::vector<std::string>& exctractedMeasurements);
    void logReadRemoteData(const std::string& frameWithMeasurements);
    void logErrReadRemoteData();
    std::string getCurrentTimeWithMilliSeconds();

private:
    boost::asio::serial_port serialPort;
    unsigned char remoteSensorData[128];

    std::ofstream outputFile;
    const std::string filePath = "sensorDataSerialReception.txt";
    static constexpr char measumrementsDelimiter = '_';

    std::vector<std::string> exctractedMeasurements{};
    bool waitForMagnData{ false };
    MeasurementsHandling measurementsHandling{ MeasurementsHandling::WAIT_FOR_ACC_GYRO };

    wxEvtHandler* m_parent;
};