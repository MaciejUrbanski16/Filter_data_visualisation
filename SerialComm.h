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

wxDEFINE_EVENT(wxEVT_MY_THREAD_EVENT_1, wxThreadEvent);

class SerialComm
{
public:
    SerialComm(boost::asio::io_context& io_context, std::string& serial_port_name, wxEvtHandler* parent) :serialPort(io_context, serial_port_name), m_parent(parent)
    {
        serialPort.set_option(boost::asio::serial_port_base::baud_rate(115200));  
        serialPort.set_option(boost::asio::serial_port_base::character_size(8)); 
        serialPort.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serialPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));

        outputFile.open(filePath, std::ios::app);

        start_receive();
    }

    ~SerialComm()
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
        try {
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
            boost::bind(&SerialComm::handle_receive,
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


            if (sizeof(remoteSensorData) != 40)
            {
                //simple logging mechanism
                //std::stringstream ss;
                //auto currentTime = getCurrentTimeWithMilliSeconds();
                //ss << currentTime << "The remoteSensorData has size different from 40 " << sizeof(remoteSensorData) << '\n';
                //outputFile << ss.str();
            }
            std::string remoteDataAsString(reinterpret_cast<char*>(remoteSensorData), sizeof(remoteSensorData));
            logReadRemoteData(remoteDataAsString);

            const std::vector<std::string> exctractedMeasurements = exctractGpsData(remoteDataAsString);

            GpsDataCustomizator* event = new GpsDataCustomizator(wxEVT_MY_THREAD_EVENT_1);
            event->SetStringVector(exctractedMeasurements);
        
            //event->SetString(strToSendToMainThread);
            //event->Set
            wxQueueEvent(m_parent, event);

            //logIntoFile(remoteDataAsString, exctractedMeasurements);
        }
        else
        {
            logErrReadRemoteData();
            std::cout << "Data not received correctly" << std::endl;
            OutputDebugStringA("Com port data received with Error!");
        }
        start_receive();
    }

    std::vector<std::string> exctractMeasurements(const std::string& frameWithMeasurements);
    std::vector<std::string>exctractGpsData(const std::string& frameWithMeasurements);
    void logIntoFile(const std::string& frameWithMeasurements, const std::vector<std::string>& exctractedMeasurements);
    void logReadRemoteData(const std::string& frameWithMeasurements);
    void logErrReadRemoteData();
    std::string getCurrentTimeWithMilliSeconds();

private:
    boost::asio::serial_port serialPort;
    unsigned char remoteSensorData[128];

    std::ofstream outputFile;
    const std::string filePath = "serialReceptionOutput.txt";
    static constexpr char measumrementsDelimiter = '_';

    wxEvtHandler* m_parent;
};