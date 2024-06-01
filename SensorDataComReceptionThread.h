#pragma once

#include <wx/thread.h>
#include "AppLogger.h"
#include "SerialComm.h"

class SensorDataComReceptionThread : public wxThread
{
public:
    SensorDataComReceptionThread(AppLogger& appLogger, wxEvtHandler* parent) : appLogger(appLogger){}
    virtual ~SensorDataComReceptionThread(){}


    virtual void* Entry();

private:
    int m_count;
    wxEvtHandler* m_parent;
    AppLogger& appLogger;
};

