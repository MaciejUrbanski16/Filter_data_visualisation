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
#include "RawMeasurements.h"

class RollPitchChartGui : public wxFrame
{
public:
	void setup(wxNotebook* m_notebook/*, MyWindow* window*/);
	void updateChart(const MeasurementsController& rawMeasurement,
					 PlotElementsBuffer& rollBasedonAccBuffer, 
					 PlotElementsBuffer& pitchBasedonAccBuffer, PlotElementsBuffer& magnPointsBuffer,
					 PlotElementsBuffer& rollBuffer,
					 PlotElementsBuffer& pitchBuffer,
					 PlotElementsBuffer& yawBuffer,
					 const double rollVal,
					 const double pitchVal,
					 const double yawVal,
					 const double filteredDistance,
					 const double timeMs);

private:
	wxStaticText* rollValue = nullptr;
	wxStaticText* pitchValue = nullptr;
	wxStaticText* yawValue = nullptr;
	wxStaticText* distanceValue = nullptr;

	wxChartPanel* rollPitchChartPanel = nullptr;
	wxSplitterWindow* rollPitchPanelSplitter = nullptr;
	wxBoxSizer* sizerRollPitchPlot = nullptr;
	double totalDistance{ 0.0 };
};

