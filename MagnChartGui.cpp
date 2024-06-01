#include "MagnChartGui.h"

void MagnChartGui::setup(wxNotebook* m_notebook /*, MyWindow* window*/)
{
	wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
	azimuthPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
	wxPanel* controlPanel = new wxPanel(azimuthPanelSplitter, wxID_ANY);

	azimuthChartPanel = new wxChartPanel(azimuthPanelSplitter);

	sizerAzimuthPlot = new wxBoxSizer(wxVERTICAL);
	azimuthChartPanel->SetMinSize(wxSize(800, 600));

	wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);

	//spinCtrlXmagn = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, 100);
	//wxStaticText* xMagnText = new wxStaticText(controlPanel, wxID_ANY, "Adjust X magn");

	//spinCtrlYmagn = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -33000, 33000, 100);
	//wxStaticText* yMagnText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Y magn");

	//controlPanelSizer->Add(xMagnText, 0, wxALL | wxALIGN_CENTER, 5);
	//controlPanelSizer->Add(spinCtrlXmagn, 0, wxALL | wxALIGN_CENTER, 5);

	//controlPanelSizer->Add(yMagnText, 0, wxALL | wxALIGN_CENTER, 5);
	//controlPanelSizer->Add(spinCtrlYmagn, 0, wxALL | wxALIGN_CENTER, 5);

	wxBoxSizer* azimuthSetupButtonsSizer = new wxBoxSizer(wxHORIZONTAL);

	startCallibrationButton = new wxButton(controlPanel, wxID_ANY, "Start callibration");
	azimuthSetupButtonsSizer->Add(startCallibrationButton, 0, wxALL | wxALIGN_LEFT, 5);
	startCallibrationButton->Bind(wxEVT_BUTTON, &MagnChartGui::OnStartMagnCallibration, this);

	stopCallibrationButton = new wxButton(controlPanel, wxID_ANY, "Stop callibration");
	azimuthSetupButtonsSizer->Add(stopCallibrationButton, 0, wxALL | wxALIGN_LEFT, 5);
	stopCallibrationButton->Bind(wxEVT_BUTTON, &MagnChartGui::OnStopMagnCallibration, this);
	stopCallibrationButton->Enable(false);

	wxBoxSizer* xMagnLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* yMagnLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* xOffsetSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* yOffsetSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* callibrateToNorthSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* callibrateToNorthValuesSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* orientationSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* checkBoxSizer = new wxBoxSizer(wxVERTICAL);

	wxStaticText* xMagnName = new wxStaticText(controlPanel, wxID_ANY, "Current X magn: ");
	wxStaticText* yMagnName = new wxStaticText(controlPanel, wxID_ANY, "Current Y magn: ");
	wxStaticText* xOffsetName = new wxStaticText(controlPanel, wxID_ANY, "Calculated X offset: ");
	wxStaticText* yOffsetName = new wxStaticText(controlPanel, wxID_ANY, "Calculated Y offset: ");
	wxStaticText* callibrateToNorthName = new wxStaticText(controlPanel, wxID_ANY, "North: ");
	wxStaticText* northName = new wxStaticText(controlPanel, wxID_ANY, "Callibrate to NORTH: ");
	wxStaticText* orientationName = new wxStaticText(controlPanel, wxID_ANY, "Orientation [deg]: ");
	spinCtrlCallibrateToNorth = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -180, 180, 0);
	spinCtrlCallibrateToNorth->Bind(wxEVT_SPINCTRL, &MagnChartGui::OnSpinToNorthCallibrate, this);
	xMagnValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	yMagnValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	callibrateToNothValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	calculatedXoffsetValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	calculatedYoffsetValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	orientationValue = new wxStaticText(controlPanel, wxID_ANY, "0");

	rawAzimuthCheckbox = new wxCheckBox(controlPanel, wxID_ANY, wxT("Plot raw azimuth"));
	rawAzimuthCheckbox->SetValue(true);
	rawAzimuthCheckbox->Bind(wxEVT_CHECKBOX, &MagnChartGui::OnRawAzimuthCheckBoxClicked, this);

	filteredAzimuthCheckbox = new wxCheckBox(controlPanel, wxID_ANY, wxT("Plot filtered azimuth"));
	filteredAzimuthCheckbox->SetValue(true);
	filteredAzimuthCheckbox->Bind(wxEVT_CHECKBOX, &MagnChartGui::OnFilteredAzimuthCheckBoxClicked, this);

	xMagnLabelsSizer->Add(xMagnName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	xMagnLabelsSizer->Add(xMagnValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(xMagnLabelsSizer, 0, wxALL, 5);

	yMagnLabelsSizer->Add(yMagnName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	yMagnLabelsSizer->Add(yMagnValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(yMagnLabelsSizer, 0, wxALL, 5);
	controlPanelSizer->AddSpacer(10);
	xOffsetSizer->Add(xOffsetName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	xOffsetSizer->Add(calculatedXoffsetValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(xOffsetSizer, 0, wxALL, 5);

	yOffsetSizer->Add(yOffsetName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	yOffsetSizer->Add(calculatedYoffsetValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(yOffsetSizer, 0, wxALL, 5);
	controlPanelSizer->AddSpacer(10);

	callibrateToNorthSizer->Add(callibrateToNorthName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	callibrateToNorthSizer->Add(spinCtrlCallibrateToNorth, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	callibrateToNorthValuesSizer->Add(northName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	callibrateToNorthValuesSizer->Add(callibrateToNothValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	orientationSizer->Add(orientationName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	orientationSizer->Add(orientationValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(callibrateToNorthSizer, 0, wxALL, 5);
	controlPanelSizer->Add(callibrateToNorthValuesSizer, 0, wxALL, 5);
	controlPanelSizer->Add(orientationSizer, 0, wxALL, 5);
	controlPanelSizer->AddSpacer(10);
	controlPanelSizer->Add(azimuthSetupButtonsSizer, 0, wxALL, 5);

	controlPanelSizer->AddSpacer(10);

	checkBoxSizer->Add(rawAzimuthCheckbox, 0, wxALL, 5);
	checkBoxSizer->Add(filteredAzimuthCheckbox, 0, wxALL, 5);
	controlPanelSizer->Add(checkBoxSizer, 0, wxALL, 5);

	controlPanel->SetSizer(controlPanelSizer);

	azimuthPanelSplitter->SplitVertically(azimuthChartPanel, controlPanel);
	sizerAzimuthPlot->Add(azimuthPanelSplitter, 1, wxEXPAND | wxALL, 5);
	panel->SetSizer(sizerAzimuthPlot);

	m_notebook->AddPage(panel, "Azimuth");
}

void MagnChartGui::updateChart(PlotElementsBuffer& magnPointsBuffer, PlotElementsBuffer& filteredAzimuthBuffer,
	PlotElementsBuffer& rollBuffer, PlotElementsBuffer& pitchBuffer, const PlotElementsBuffer& expectedOrientationBuffer,
	const PlotElementsBuffer& gpsAzimuthBuffer,
	const int16_t xMagn, const int16_t yMagn, const double azimuth, const double filteredAzimuth,
	 const double timeMs)
{
	if (wasCallibrationStarted)
	{
		magnetometerCallibrator.collectData(xMagn, yMagn);
	}

	xMagnValue->SetLabel(std::to_string(xMagn));
	yMagnValue->SetLabel(std::to_string(yMagn));
	orientationValue->SetLabel(std::to_string(azimuth));

	magnPointsBuffer.AddElement(wxRealPoint(timeMs, azimuth));
	filteredAzimuthBuffer.AddElement(wxRealPoint(timeMs, filteredAzimuth));

	XYPlot* plot = new XYPlot();
	XYSimpleDataset* dataset = new XYSimpleDataset();

	//dataset->AddSerie(new XYSerie(rollBuffer.getBuffer()));
	//dataset->GetSerie(0)->SetName("roll");
	//dataset->AddSerie(new XYSerie(pitchBuffer.getBuffer()));
	//dataset->GetSerie(1)->SetName("pitch");

	if (plotFilteredAzimuth and plotRawAzimuth)
	{
		dataset->AddSerie(new XYSerie(magnPointsBuffer.getBuffer()));
		dataset->AddSerie(new XYSerie(filteredAzimuthBuffer.getBuffer()));
		dataset->AddSerie(new XYSerie(expectedOrientationBuffer.getBuffer()));
		dataset->AddSerie(new XYSerie(gpsAzimuthBuffer.getBuffer()));
		dataset->GetSerie(0)->SetName("Raw azimuth");
		dataset->GetSerie(1)->SetName("Filtered azimuth");
		dataset->GetSerie(2)->SetName("Expected azimuth");
		dataset->GetSerie(3)->SetName("Gps azimuth");
	}
	else if (plotFilteredAzimuth and not plotRawAzimuth)
	{
		dataset->AddSerie(new XYSerie(filteredAzimuthBuffer.getBuffer()));
		dataset->AddSerie(new XYSerie(expectedOrientationBuffer.getBuffer()));
		dataset->AddSerie(new XYSerie(gpsAzimuthBuffer.getBuffer()));
		dataset->GetSerie(0)->SetName("Filtered azimuth");
		dataset->GetSerie(1)->SetName("Expected azimuth");
		dataset->GetSerie(2)->SetName("Gps azimuth");

	}
	else if(not plotFilteredAzimuth and plotRawAzimuth)
	{
		dataset->AddSerie(new XYSerie(magnPointsBuffer.getBuffer()));
		dataset->AddSerie(new XYSerie(expectedOrientationBuffer.getBuffer()));
		dataset->AddSerie(new XYSerie(gpsAzimuthBuffer.getBuffer()));
		dataset->GetSerie(0)->SetName("Raw azimuth");
		dataset->GetSerie(1)->SetName("Expected azimuth");
		dataset->GetSerie(2)->SetName("Gps azimuth");
	}

	//dataset->AddSerie(new XYSerie(accPoints));
	dataset->SetRenderer(new XYLineRenderer());
	NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
	NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
	leftAxis->SetTitle(wxT("Azimuth [deg]"));
	bottomAxis->SetTitle(wxT("Time [ms]"));
	if (magnPointsBuffer.getBuffer().size() >= 100)
	{
		bottomAxis->SetFixedBounds(magnPointsBuffer.getBuffer()[0].x, magnPointsBuffer.getBuffer()[99].x);
	}
	Legend* legend = new Legend(wxTOP, wxLEFT);
	plot->SetLegend(legend);
	plot->AddObjects(dataset, leftAxis, bottomAxis);

	Chart* chart = new Chart(plot, "Azimuth");

	azimuthChartPanel->SetChart(chart);
}

void MagnChartGui::setCalculatedOffsetValues()
{
	calculatedXoffsetValue->SetLabel(std::to_string(magnetometerCallibrator.getXoffset()));
	calculatedYoffsetValue->SetLabel(std::to_string(magnetometerCallibrator.getYoffset()));
}
void MagnChartGui::OnSpinToNorthCallibrate(wxSpinEvent& event)
{
	if (not wasCallibrationStarted)
	{
		const int bias = spinCtrlCallibrateToNorth->GetValue();
		biasToNorth = bias;
		magnetometerCallibrator.setBiasToNorth(biasToNorth);
	}
	else
	{
		wxMessageBox("magnetometer offsets have not been calculated yet.", "Informacja", wxOK | wxICON_INFORMATION);
	}

}

void MagnChartGui::OnStartMagnCallibration(wxCommandEvent& event)
{
	wasCallibrationStarted = true;
	startCallibrationButton->Enable(false);
	stopCallibrationButton->Enable(true);
	wxMessageBox("Callibration has been started, rotate device to collect data", "Informacja", wxOK | wxICON_INFORMATION);
}

void MagnChartGui::OnStopMagnCallibration(wxCommandEvent& event)
{
	wasCallibrationStarted = false;
	wasMagnCallibrationDone = true;
	magnetometerCallibrator.callibrate();
	setCalculatedOffsetValues();
	stopCallibrationButton->Enable(false);
	wxMessageBox("Callibration has been finished", "Informacja", wxOK | wxICON_INFORMATION);
}

void MagnChartGui::OnRawAzimuthCheckBoxClicked(wxCommandEvent& event)
{
	if (rawAzimuthCheckbox->IsChecked())
	{
		plotRawAzimuth = true;
		wxMessageBox("Raw azimuth checkbox zosta³ zaznaczony!", "Informacja", wxOK | wxICON_INFORMATION, this);
	}
	else
	{
		plotRawAzimuth = false;
		wxMessageBox("Raw azimuth checkbox zosta³ odznaczony!", "Informacja", wxOK | wxICON_INFORMATION, this);
	}
}

void MagnChartGui::OnFilteredAzimuthCheckBoxClicked(wxCommandEvent& event)
{
	if (filteredAzimuthCheckbox->IsChecked())
	{
		plotFilteredAzimuth = true;
		wxMessageBox("Filtered azimuth checkbox zosta³ zaznaczony!", "Informacja", wxOK | wxICON_INFORMATION, this);
	}
	else
	{
		plotFilteredAzimuth = false;
		wxMessageBox("Filtered azimuth checkbox zosta³ odznaczony!", "Informacja", wxOK | wxICON_INFORMATION, this);
	}
}