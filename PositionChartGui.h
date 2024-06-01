#pragma once

#include <wx/wx.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/grid.h>
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

#include "PlotElementsBuffer.h"

class PositionChartGui
{
public:
	void setup(wxNotebook* m_notebook);
	void updateChart(PlotElementsBuffer& rawPositionBuffer, const double xDistance, const double yDistance, const double timeMs);

private:
	wxChartPanel* positionChartPanel = nullptr;

	double currentXPos{ 0.0 };
	double currentYPos{ 0.0 };
};

