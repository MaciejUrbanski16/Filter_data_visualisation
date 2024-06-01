#include "AzimuthDiffChartGui.h"

void AzimuthDiffChartGui::setup(wxNotebook* m_notebook)
{
	wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
	azimuthDiffPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
	wxPanel* controlPanel = new wxPanel(azimuthDiffPanelSplitter, wxID_ANY);

	azimuthDiffChartPanel = new wxChartPanel(azimuthDiffPanelSplitter);

	sizerAzimuthDiffPlot = new wxBoxSizer(wxVERTICAL);
	azimuthDiffChartPanel->SetMinSize(wxSize(800, 600));

	wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);
	wxBoxSizer* azimuthDiffExpectedVsFilteredSizer = new wxBoxSizer(wxHORIZONTAL);

	wxStaticText* azimuthDiffExpectedVsFilteredName = new wxStaticText(controlPanel, wxID_ANY, "Azimuth difference expected vs filtered: ");
	azimuthDiffExpectedVsFilteredValue = new wxStaticText(controlPanel, wxID_ANY, "0");

	azimuthDiffExpectedVsFilteredSizer->Add(azimuthDiffExpectedVsFilteredName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	azimuthDiffExpectedVsFilteredSizer->Add(azimuthDiffExpectedVsFilteredValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(azimuthDiffExpectedVsFilteredSizer, 0, wxALL, 5);

	controlPanel->SetSizer(controlPanelSizer);

	azimuthDiffPanelSplitter->SplitVertically(azimuthDiffChartPanel, controlPanel);
	sizerAzimuthDiffPlot->Add(azimuthDiffPanelSplitter, 1, wxEXPAND | wxALL, 5);
	panel->SetSizer(sizerAzimuthDiffPlot);

	m_notebook->AddPage(panel, "Azimuth DIFF");
}

void AzimuthDiffChartGui::updateChart(const PlotElementsBuffer& azimuthDiff)
{
	XYPlot* plot = new XYPlot();
	XYSimpleDataset* dataset = new XYSimpleDataset();
	azimuthDiffExpectedVsFilteredValue->SetLabel(std::to_string(std::fabs(azimuthDiff.getBuffer().back().y)));

	dataset->AddSerie(new XYSerie(azimuthDiff.getBuffer()));
	dataset->GetSerie(0)->SetName("Azimuth difference expected vs filtered");

	dataset->SetRenderer(new XYLineRenderer());
	NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
	NumberAxis* rightAxis = new NumberAxis(AXIS_RIGHT);
	NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
	leftAxis->SetTitle(wxT("Azimuth difference [deg]"));
	bottomAxis->SetTitle(wxT("Time [ms]"));
	if (azimuthDiff.getBuffer().size() >= 1000)
	{
		bottomAxis->SetFixedBounds(azimuthDiff.getBuffer()[0].x, azimuthDiff.getBuffer()[999].x);
	}
	Legend* legend = new Legend(wxTOP, wxRIGHT);
	plot->SetLegend(legend);
	plot->AddObjects(dataset, leftAxis, bottomAxis);
	//plot->AddObjects(dataset, rightAxis, bottomAxis);

	Chart* chart = new Chart(plot, "Azimuth difference");
	azimuthDiffChartPanel->SetChart(chart);
}
