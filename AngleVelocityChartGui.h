#pragma once

#include <wx/wx.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/chartpanel.h>
#include <wx/chart.h>
#include <wx/chartype.h>
#include <wx/wxfreechartdefs.h>
#include <wx/legend.h>
#include <wx/dataset.h>
#include <wx/colorscheme.h>
#include <wx/category/categorydataset.h>
#include <wx/xy/xyplot.h>
#include <wx/xy/xylinerenderer.h>
#include <wx/xy/xysimpledataset.h>
#include <wx/xy/vectordataset.h>
#include <wx/splitter.h>
#include <wx/spinctrl.h> 

#include "PlotElementsBuffer.h"

class AngleVelocityChartGui : public wxFrame
{
public:
	void setup(wxNotebook* m_notebook);
	void updateChart(PlotElementsBuffer& xAngleVelocityBuffer, PlotElementsBuffer& yAngleVelocityBuffer,
        PlotElementsBuffer& zAngleVelocityBuffer, PlotElementsBuffer& filteredZangleVelocityBuffer,
        const double xAngleVel, const double yAngleVel, const double zAngleVel, const double filteredZangleVelocity, const double timeMs);

    double getXgyroBias() const;
    double getYgyroBias() const;
    double getZgyroBias() const;

private:
    void OnResetAngleVelChart(wxCommandEvent& event);
    void OnSubmitAngleVelAdjustments(wxCommandEvent& event);
    void OnSpinXAngleVelUpdate(wxSpinEvent& event);
    void OnSpinXAnglVelIncrUpdate(wxSpinEvent& event);
    void OnSpinYAngleVelUpdate(wxSpinEvent& event);
    void OnSpinYAnglVelIncrUpdate(wxSpinEvent& event);
    void OnSpinZAngleVelUpdate(wxSpinEvent& event);
    void OnSpinZAnglVelIncrUpdate(wxSpinEvent& event);

    void OnRawXangleVelCheckBoxClicked(wxCommandEvent& event);
    void OnRawYangleVelCheckBoxClicked(wxCommandEvent& event);
    void OnRawZangleVelCheckBoxClicked(wxCommandEvent& event);
    void OnFilteredZangleVelCheckBoxClicked(wxCommandEvent& event);

    wxChartPanel* angleVelocityChartPanel = nullptr;
    wxSplitterWindow* angleVelPanelSplitter = nullptr;
    wxBoxSizer* sizerAngleVelPlot = nullptr;
    wxSpinCtrl* spinCtrlXangleVel = nullptr;
    wxSpinCtrl* spinCtrlXangleVelMultiplicator = nullptr;
    wxSpinCtrl* spinCtrlYangleVel = nullptr;
    wxSpinCtrl* spinCtrlYangleVelMultiplicator = nullptr;
    wxSpinCtrl* spinCtrlZangleVel = nullptr;
    wxSpinCtrl* spinCtrlZangleVelMultiplicator = nullptr;

    wxStaticText* xAngleVelValue = nullptr;
    wxStaticText* yAngleVelValue = nullptr;
    wxStaticText* zAngleVelValue = nullptr;
    wxCheckBox* rawXangleVelCheckbox = nullptr;
    wxCheckBox* rawYangleVelCheckbox = nullptr;
    wxCheckBox* rawZangleVelCheckbox = nullptr;
    wxCheckBox* filteredZangleVelCheckbox = nullptr;

    bool plotRawXangleVelocity{ true };
    bool plotRawYangleVelocity{ true };
    bool plotRawZangleVelocity{ true };
    bool plotFilteredZangleVelocity{ true };

    double xGyroBias{ -18000.0 };
    double yGyroBias{ 12900.0 };
    double zGyroBias{ 9000.0 };
};

