#pragma once

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <vector>

#include "MeasurementCustomizator.h"
#include "GpsDataCustomizator.h"
#include "AppLogger.h"

using namespace boost::asio;

wxDEFINE_EVENT(wxEVT_MY_THREAD_EVENT, wxThreadEvent);

class Session : public std::enable_shared_from_this<Session> {
public:
    Session(ip::tcp::socket socket, AppLogger& appLogger, wxEvtHandler* parent) : socket_(std::move(socket)),
        appLogger(appLogger), m_parent(parent) {}

    void start() {
        doRead();
    }

private:
    void doRead() {
        auto self(shared_from_this());

        socket_.async_read_some(buffer(data_),
            [this, self](boost::system::error_code ec, std::size_t length) {
                if (!ec)
                {
                    const std::string receivedMeasurement = std::string(data_.begin(), data_.begin() + length);
                    const char separator = 'l';
                    int numOfSeparators = std::count(receivedMeasurement.begin(), receivedMeasurement.end(), separator);
                    if (numOfSeparators == 9)
                    {
                        std::stringstream ss;
                        ss << "Session on: " << socket_.remote_endpoint().address().to_string() << " : port: " << socket_.remote_endpoint().port() <<
                            " Received data from client: " << receivedMeasurement;
                        const std::string msg = ss.str();

                        appLogger.logReceivedDataOnTcpPort(msg);

                        //DATA STRUCTURE Xacc 6 elements
                        //               Yacc 6 elements
                        //               Zacc 6 elements
                        //               Xgyr 6 elements
                        //               Ygyr 6 elements 
                        //               Zgyr 6 elements 
                        //               Xmagn 6 elements
                        //               Ymagn 6 elements
                        //               Zmagn 6 elements

                        const std::vector<std::string> exctractedMeasurements = exctractMeasurements(receivedMeasurement);

                        MeasurementCustomizator* event = new MeasurementCustomizator(wxEVT_MY_THREAD_EVENT);
                        event->SetStringVector(exctractedMeasurements);
                        wxQueueEvent(m_parent, event);

                        doRead();
                    }
                    else if(numOfSeparators == 4)
                    {
                        std::stringstream ss;
                        ss << "Session on: " << socket_.remote_endpoint().address().to_string() << " : port: " << socket_.remote_endpoint().port() <<
                            " Received GPS data from client: " << receivedMeasurement;
                        const std::string msg = ss.str();

                        appLogger.logReceivedDataOnTcpPort(msg);

                        const std::vector<std::string> exctractedGpsMeasurements = exctractMeasurements(receivedMeasurement);

                        GpsDataCustomizator* event = new GpsDataCustomizator(wxEVT_MY_THREAD_EVENT);
                        event->SetStringVector(exctractedGpsMeasurements);
                        wxQueueEvent(m_parent, event);

                        doRead();
                    }
                    else
                    {
                        std::stringstream ss;
                        ss << "ERR Session on: " << socket_.remote_endpoint().address().to_string() << " : port: " << socket_.remote_endpoint().port() <<
                            " Received invalid data from client: " << receivedMeasurement << " - numOfSeparators: " << numOfSeparators;
                        const std::string msg = ss.str();

                        appLogger.logReceivedDataOnTcpPort(msg);
                        doRead();
                    }

                }

            }
        );
    }

    std::vector<std::string> exctractMeasurements(const std::string & frameWithMeasurements)
    {
        std::istringstream ss(frameWithMeasurements);
        std::vector<std::string> tokens;
        std::string token;
        int nr = 0;
        while (std::getline(ss, token, measumrementsDelimiter))
        {
            if (nr < 9)
            {
                nr++;
                tokens.push_back(token);
            }
        }
        return tokens;
    }

    std::vector<std::string> exctractGpsMeasurements(const std::string& frameWithMeasurements)
    {
        std::istringstream ss(frameWithMeasurements);
        std::vector<std::string> tokens;
        std::string token;
        int nr = 0;
        while (std::getline(ss, token, measumrementsDelimiter))
        {
            if (nr < 5)
            {
                nr++;
                tokens.push_back(token);
            }
        }
        return tokens;
    }

    ip::tcp::socket socket_;
    boost::array<char, 128> data_;

    wxEvtHandler* m_parent;
    AppLogger& appLogger;

    static constexpr char measumrementsDelimiter = '_';
};

class Server {
public:
    Server(io_service& io, short port, AppLogger& appLogger, wxEvtHandler* parent) : acceptor_(io, ip::tcp::endpoint(ip::tcp::v4(), port)), appLogger(appLogger) {
        doAccept(parent);
    }

private:
    void doAccept(wxEvtHandler* parent) {
        acceptor_.async_accept(
            [this, parent](boost::system::error_code ec, ip::tcp::socket socket) {
                if (!ec) 
                {
                    std::make_shared<Session>(std::move(socket), appLogger, parent)->start();
                }

                doAccept(parent);
            }
        );
    }

    ip::tcp::acceptor acceptor_;
    AppLogger& appLogger;

};