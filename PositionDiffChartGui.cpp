#include "PositionDiffChartGui.h"

void PositionDiffChartGui::setup(wxNotebook* m_notebook)
{
	wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
	positionDiffPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
	wxPanel* controlPanel = new wxPanel(positionDiffPanelSplitter, wxID_ANY);

	positionDiffChartPanel = new wxChartPanel(positionDiffPanelSplitter);

	sizerPositionDiffPlot = new wxBoxSizer(wxVERTICAL);
	positionDiffChartPanel->SetMinSize(wxSize(800, 600));

	wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);
	wxBoxSizer* positionDiffExpectedVsFilteredSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* positionDiffExpectedVsGpsSizer = new wxBoxSizer(wxHORIZONTAL);

	wxStaticText* positionDiffExpectedVsFilteredName = new wxStaticText(controlPanel, wxID_ANY, "Position diff expected vs filtered: ");
	positionDiffExpectedVsFilteredValue = new wxStaticText(controlPanel, wxID_ANY, "0");

	wxStaticText* positionDiffExpectedVsGpsName = new wxStaticText(controlPanel, wxID_ANY, "Position diff expected vs GPS: ");
	positionDiffExpectedVsGpsValue = new wxStaticText(controlPanel, wxID_ANY, "0");

	positionDiffExpectedVsFilteredSizer->Add(positionDiffExpectedVsFilteredName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	positionDiffExpectedVsFilteredSizer->Add(positionDiffExpectedVsFilteredValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(positionDiffExpectedVsFilteredSizer, 0, wxALL, 5);

	positionDiffExpectedVsGpsSizer->Add(positionDiffExpectedVsGpsName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	positionDiffExpectedVsGpsSizer->Add(positionDiffExpectedVsGpsValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(positionDiffExpectedVsGpsSizer, 0, wxALL, 5);

	controlPanel->SetSizer(controlPanelSizer);

	positionDiffPanelSplitter->SplitVertically(positionDiffChartPanel, controlPanel);
	sizerPositionDiffPlot->Add(positionDiffPanelSplitter, 1, wxEXPAND | wxALL, 5);
	panel->SetSizer(sizerPositionDiffPlot);

	m_notebook->AddPage(panel, "Position DIFF");
}

void PositionDiffChartGui::updateChart(const PlotElementsBuffer& positionDiff, const PlotElementsBuffer& positionDiffExpectedVsGps)
{
	XYPlot* plot = new XYPlot();
	XYSimpleDataset* dataset = new XYSimpleDataset();
	positionDiffExpectedVsFilteredValue->SetLabel(std::to_string(positionDiff.getBuffer().back().y));
	positionDiffExpectedVsGpsValue->SetLabel(std::to_string(positionDiffExpectedVsGps.getBuffer().back().y));

	dataset->AddSerie(new XYSerie(positionDiff.getBuffer()));
	dataset->GetSerie(0)->SetName("Position difference expected vs filtered");

	dataset->AddSerie(new XYSerie(positionDiffExpectedVsGps.getBuffer()));
	dataset->GetSerie(1)->SetName("Position difference expected vs gps");

	dataset->SetRenderer(new XYLineRenderer());
	NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
	NumberAxis* rightAxis = new NumberAxis(AXIS_RIGHT);
	NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
	leftAxis->SetTitle(wxT("Position difference [m]"));
	rightAxis->SetTitle(wxT("Azimuth difference [deg]"));
	bottomAxis->SetTitle(wxT("Time [ms]"));
	if (positionDiff.getBuffer().size() >= 1000)
	{
		bottomAxis->SetFixedBounds(positionDiff.getBuffer()[0].x, positionDiff.getBuffer()[999].x);
	}
	Legend* legend = new Legend(wxTOP, wxRIGHT);
	plot->SetLegend(legend);
	plot->AddObjects(dataset, leftAxis, bottomAxis);

	Chart* chart = new Chart(plot, "Position difference");
	positionDiffChartPanel->SetChart(chart);
}
