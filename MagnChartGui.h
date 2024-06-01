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
#include "MagnetometerCallibrator.h"

class MagnChartGui : public wxFrame
{
public:
	MagnChartGui(MagnetometerCallibrator& magnetometerCallibrator) : magnetometerCallibrator(magnetometerCallibrator){}
	void setup(wxNotebook* m_notebook/*, MyWindow* window*/);
	void updateChart(PlotElementsBuffer& magnPointsBuffer, PlotElementsBuffer& filteredAzimuthBuffer,
		PlotElementsBuffer& rollBuffer, PlotElementsBuffer& pitchBuffer, const PlotElementsBuffer& expectedOrientationBuffer,
		const PlotElementsBuffer& gpsAzimuthBuffer,
		 const int16_t xMagn, const int16_t yMagn, const double azimuth, const double filteredAzimuth,
		 const double timeMs);
	bool checkIfMagnCalibrationDone() const { return wasMagnCallibrationDone; }

private:
	void OnStartMagnCallibration(wxCommandEvent& event);
	void OnStopMagnCallibration(wxCommandEvent& event);
	void OnRawAzimuthCheckBoxClicked(wxCommandEvent& event);
	void OnFilteredAzimuthCheckBoxClicked(wxCommandEvent& event);
	void OnSpinToNorthCallibrate(wxSpinEvent& event);
	void setCalculatedOffsetValues();
	//void OnSpinMagnUpdate(wxSpinEvent& event);
	//void OnSpinMagnIncrUpdate(wxSpinEvent& event);

	wxChartPanel* azimuthChartPanel = nullptr;
	wxSplitterWindow* azimuthPanelSplitter = nullptr;
	wxBoxSizer* sizerAzimuthPlot = nullptr;
	wxSpinCtrl* spinCtrlXmagn = nullptr;
	wxSpinCtrl* spinCtrlYmagn = nullptr;
	wxSpinCtrl* spinCtrlCallibrateToNorth = nullptr;
	wxStaticText* callibrateToNothValue = nullptr;
	wxStaticText* xMagnValue = nullptr;
	wxStaticText* yMagnValue = nullptr;
	wxStaticText* calculatedXoffsetValue = nullptr;
	wxStaticText* calculatedYoffsetValue = nullptr;
	wxStaticText* orientationValue = nullptr;
	wxCheckBox* rawAzimuthCheckbox = nullptr;
	wxCheckBox* filteredAzimuthCheckbox = nullptr;

	wxButton* startCallibrationButton = nullptr;
	wxButton* stopCallibrationButton = nullptr;

	bool plotRawAzimuth{ true };
	bool plotFilteredAzimuth{ true };

	bool wasCallibrationStarted{ false };
	bool wasMagnCallibrationDone{ false };
	double currentAzimuth{ 0.0 };
	int16_t biasToNorth{ 0 };
	MagnetometerCallibrator& magnetometerCallibrator;

};
