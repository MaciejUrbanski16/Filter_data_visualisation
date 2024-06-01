#include "VelocityChartGui.h"

void VelocityChartGui::setup(wxNotebook* m_notebook)
{
	wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
	velocityChartPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
	wxPanel* controlPanel = new wxPanel(velocityChartPanelSplitter, wxID_ANY);

	velocityChartPanel = new wxChartPanel(velocityChartPanelSplitter);

	sizerVelocityPlot = new wxBoxSizer(wxVERTICAL);
	velocityChartPanel->SetMinSize(wxSize(800, 600));

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

	wxButton* resetButton = new wxButton(controlPanel, wxID_ANY, "Reset chart");
	azimuthSetupButtonsSizer->Add(resetButton, 0, wxALL | wxALIGN_LEFT, 5);
	//resetButton->Bind(wxEVT_BUTTON, &MagnChartGui::OnResetMagnChart, this);

	wxButton* submitButton = new wxButton(controlPanel, wxID_ANY, "Submit adjustments");
	azimuthSetupButtonsSizer->Add(submitButton, 0, wxALL | wxALIGN_LEFT, 5);
	//submitButton->Bind(wxEVT_BUTTON, &MagnChartGui::OnSubmitMagnAdjustments, this);

	wxBoxSizer* velocityLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* velocityFilteredLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* actualDistanceLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	//wxBoxSizer* pitchLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	//wxBoxSizer* yawLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	//wxBoxSizer* distanceLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	//wxBoxSizer* orientationSizer = new wxBoxSizer(wxHORIZONTAL);
	//wxBoxSizer* checkBoxSizer = new wxBoxSizer(wxVERTICAL);

	wxStaticText* velocityName = new wxStaticText(controlPanel, wxID_ANY, "Velocity[m/s]: ");
	wxStaticText* velocityFilteredName = new wxStaticText(controlPanel, wxID_ANY, "Velocity filtered[m/s]: ");
	wxStaticText* actualDistnaceName = new wxStaticText(controlPanel, wxID_ANY, "Actual distance[m]: ");
	//wxStaticText* pitchName = new wxStaticText(controlPanel, wxID_ANY, "Pitch: ");
	//wxStaticText* yawName = new wxStaticText(controlPanel, wxID_ANY, "Yaw: ");
	//wxStaticText* distanceName = new wxStaticText(controlPanel, wxID_ANY, "Distance: ");
	//wxStaticText* orientationName = new wxStaticText(controlPanel, wxID_ANY, "Orientation [deg]: ");
	velocityValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	velocityFilteredValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	actualDistanceValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	//pitchValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	//yawValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	//distanceValue = new wxStaticText(controlPanel, wxID_ANY, "0");
	//orientationValue = new wxStaticText(controlPanel, wxID_ANY, "0");

	//rawAzimuthCheckbox = new wxCheckBox(controlPanel, wxID_ANY, wxT("Plot raw azimuth"));
	//rawAzimuthCheckbox->SetValue(true);
	//rawAzimuthCheckbox->Bind(wxEVT_CHECKBOX, &MagnChartGui::OnRawAzimuthCheckBoxClicked, this);

	//filteredAzimuthCheckbox = new wxCheckBox(controlPanel, wxID_ANY, wxT("Plot filtered azimuth"));
	//filteredAzimuthCheckbox->SetValue(true);
	//filteredAzimuthCheckbox->Bind(wxEVT_CHECKBOX, &MagnChartGui::OnFilteredAzimuthCheckBoxClicked, this);

	velocityLabelsSizer->Add(velocityName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	velocityLabelsSizer->Add(velocityValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(velocityLabelsSizer, 0, wxALL, 5);

	velocityFilteredLabelsSizer->Add(velocityFilteredName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	velocityFilteredLabelsSizer->Add(velocityFilteredValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(velocityFilteredLabelsSizer, 0, wxALL, 5);

	actualDistanceLabelsSizer->Add(actualDistnaceName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	actualDistanceLabelsSizer->Add(actualDistanceValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(actualDistanceLabelsSizer, 0, wxALL, 5);

	//pitchLabelsSizer->Add(pitchName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	//pitchLabelsSizer->Add(pitchValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	//controlPanelSizer->Add(pitchLabelsSizer, 0, wxALL, 5);

	//yawLabelsSizer->Add(yawName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	//yawLabelsSizer->Add(yawValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	//controlPanelSizer->Add(yawLabelsSizer, 0, wxALL, 5);

	//distanceLabelsSizer->Add(distanceName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	//distanceLabelsSizer->Add(distanceValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	//controlPanelSizer->Add(distanceLabelsSizer, 0, wxALL, 5);

	//orientationSizer->Add(orientationName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	//orientationSizer->Add(orientationValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	//controlPanelSizer->Add(orientationSizer, 0, wxALL, 5);

	controlPanelSizer->Add(azimuthSetupButtonsSizer, 0, wxALL, 5);

	controlPanelSizer->AddSpacer(10);

	//checkBoxSizer->Add(rawAzimuthCheckbox, 0, wxALL, 5);
	//checkBoxSizer->Add(filteredAzimuthCheckbox, 0, wxALL, 5);
	//controlPanelSizer->Add(checkBoxSizer, 0, wxALL, 5);

	controlPanel->SetSizer(controlPanelSizer);

	velocityChartPanelSplitter->SplitVertically(velocityChartPanel, controlPanel);
	sizerVelocityPlot->Add(velocityChartPanelSplitter, 1, wxEXPAND | wxALL, 5);
	panel->SetSizer(sizerVelocityPlot);

	m_notebook->AddPage(panel, "Velocity");
}

void VelocityChartGui::updateChart(PlotElementsBuffer& calculatedVelocityBuffer,
	PlotElementsBuffer& filteredVelocityBuffer, const double actualVelocity, const double actualVelocityFromFilter,
	const double actualDistance)
{
	velocityValue->SetLabel(std::to_string(actualVelocity));
	velocityFilteredValue->SetLabel(std::to_string(actualVelocityFromFilter));
	actualDistanceValue->SetLabel(std::to_string(actualDistance));

	XYPlot* plot = new XYPlot();
	XYSimpleDataset* dataset = new XYSimpleDataset();

	dataset->AddSerie(new XYSerie(calculatedVelocityBuffer.getBuffer()));
	dataset->GetSerie(0)->SetName("velocity");
	dataset->AddSerie(new XYSerie(filteredVelocityBuffer.getBuffer()));
	dataset->GetSerie(0)->SetName("filtered velocity");

	dataset->SetRenderer(new XYLineRenderer());
	NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
	NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
	leftAxis->SetTitle(wxT("Velocity [m/s]"));
	bottomAxis->SetTitle(wxT("Time [ms]"));
	if (calculatedVelocityBuffer.getBuffer().size() >= 100)
	{
		bottomAxis->SetFixedBounds(calculatedVelocityBuffer.getBuffer()[0].x, calculatedVelocityBuffer.getBuffer()[99].x);
	}
	Legend* legend = new Legend(wxTOP, wxRIGHT);
	plot->SetLegend(legend);
	plot->AddObjects(dataset, leftAxis, bottomAxis);

	Chart* chart = new Chart(plot, "Roll/pitch/yaw");
	velocityChartPanel->SetChart(chart);
}
