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

class AzimuthDiffChartGui
{
public:
	void setup(wxNotebook* m_notebook);
	void updateChart(const PlotElementsBuffer& azimuthDiff);

private:
	wxChartPanel* azimuthDiffChartPanel = nullptr;
	wxSplitterWindow* azimuthDiffPanelSplitter = nullptr;
	wxBoxSizer* sizerAzimuthDiffPlot = nullptr;

	wxStaticText* azimuthDiffExpectedVsFilteredValue = nullptr;
};

