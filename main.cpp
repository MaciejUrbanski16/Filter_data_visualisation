#include "MainWindow.h"

class MyApp : public wxApp
{
public:
    virtual bool OnInit();
};

IMPLEMENT_APP(MyApp)
bool MyApp::OnInit()
{
    MyWindow* frame = new MyWindow(wxT("Filter data visualisation"));
    frame->Show(true);

    return true;
}