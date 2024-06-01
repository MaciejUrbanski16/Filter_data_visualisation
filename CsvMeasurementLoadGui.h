#pragma once

#include <wx/wx.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/grid.h>
#include <wx/sizer.h>
#include "CsvMeasurementReader.h"
#include "DeltaTimeCalculator.h"

class CsvMeasurementLoadGui
{
public:
	CsvMeasurementLoadGui(CsvMeasurementReader& csvMeasurementReader, wxTimer& filterFileMeasTimer, wxTimer& filterFileGpsTimer, DeltaTimeCalculator& deltaTimeCalculator) :
		csvMeasurementReader(csvMeasurementReader), filterFileMeasTimer(filterFileMeasTimer), filterFileGpsTimer(filterFileGpsTimer), deltaTimeCalculator(deltaTimeCalculator)
	{}

	void setup(wxPanel* kalmanParamsSetupPanel);

private:
	void OnLoadSensorData(wxCommandEvent& event);
	void OnLoadGpsData(wxCommandEvent& event);
	void OnLoadExpectedPositionData(wxCommandEvent& event);
	void OnStartFiltration(wxCommandEvent& event);

	bool isFileExtensionCorrect(const wxString& filePath, const wxString& extension) const;

	wxButton* loadSensorsDataFromFileButton = nullptr;
	wxButton* loadGpsDataFromFileButton = nullptr;
	wxButton* loadExpectedPositionDataFromFileButton = nullptr;
	wxButton* startFiltrationButton = nullptr;

	wxTextCtrl* filePathSensorDataTextCtrl = nullptr;
	wxTextCtrl* filePathGpsDataTextCtrl = nullptr;
	wxTextCtrl* fileExpectedPositionDataTextCtrl = nullptr;
	wxFileDialog* openFileDialog = nullptr;

	wxTimer& filterFileMeasTimer;
	wxTimer& filterFileGpsTimer;
	CsvMeasurementReader& csvMeasurementReader;
	DeltaTimeCalculator& deltaTimeCalculator;

	bool sensorDataPathGiven{ false };
	bool gpsDataPathGiven{ false };
	bool canFiltrationBeStarted{ false };
};

