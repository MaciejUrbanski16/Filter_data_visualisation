#pragma once

#include <wx/thread.h>
#include "AppLogger.h"
#include "SerialComm.h"

class GpsDataReceptionThread : public wxThread
{
public:
    GpsDataReceptionThread(AppLogger& appLogger, wxEvtHandler* parent);
    virtual ~GpsDataReceptionThread();


    virtual void* Entry();

private:
    int m_count;
    wxEvtHandler* m_parent;
    AppLogger& appLogger;
};
