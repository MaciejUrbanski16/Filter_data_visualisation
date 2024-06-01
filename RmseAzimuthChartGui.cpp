#include "RmseAzimuthChartGui.h"

void RmseAzimuthChartGui::setup(wxNotebook* m_notebook)
{
	wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
	rmsePanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
	wxPanel* controlPanel = new wxPanel(rmsePanelSplitter, wxID_ANY);

	rmseChartPanel = new wxChartPanel(rmsePanelSplitter);

	sizerRmsePlot = new wxBoxSizer(wxVERTICAL);
	rmseChartPanel->SetMinSize(wxSize(800, 600));

	wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);
	wxBoxSizer* rmseAzimuthLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* rmsePositionXLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* rmsePositionYLabelsSizer = new wxBoxSizer(wxHORIZONTAL);

	wxStaticText* rmseAzimuthName = new wxStaticText(controlPanel, wxID_ANY, "Rmse azimuth: ");
	rmseAzimuthValue = new wxStaticText(controlPanel, wxID_ANY, "0");

	wxStaticText* rmsePositionXName = new wxStaticText(controlPanel, wxID_ANY, "Rmse position X: ");
	rmsePositionXValue = new wxStaticText(controlPanel, wxID_ANY, "0");

	wxStaticText* rmsePositionYName = new wxStaticText(controlPanel, wxID_ANY, "Rmse position Y: ");
	rmsePositionYValue = new wxStaticText(controlPanel, wxID_ANY, "0");

	rmseAzimuthLabelsSizer->Add(rmseAzimuthName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	rmseAzimuthLabelsSizer->Add(rmseAzimuthValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(rmseAzimuthLabelsSizer, 0, wxALL, 5);

	rmsePositionXLabelsSizer->Add(rmsePositionXName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	rmsePositionXLabelsSizer->Add(rmsePositionXValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(rmsePositionXLabelsSizer, 0, wxALL, 5);

	rmsePositionYLabelsSizer->Add(rmsePositionYName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	rmsePositionYLabelsSizer->Add(rmsePositionYValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
	controlPanelSizer->Add(rmsePositionYLabelsSizer, 0, wxALL, 5);

	controlPanel->SetSizer(controlPanelSizer);

	rmsePanelSplitter->SplitVertically(rmseChartPanel, controlPanel);
	sizerRmsePlot->Add(rmsePanelSplitter, 1, wxEXPAND | wxALL, 5);
	panel->SetSizer(sizerRmsePlot);

	m_notebook->AddPage(panel, "RMSE");
}

void RmseAzimuthChartGui::updateChart(const PlotElementsBuffer& rmseAzimuthBuffer, const PlotElementsBuffer& rmsePositionXBuffer
	, const PlotElementsBuffer& rmsePositionYBuffer)
{

	rmseAzimuthValue->SetLabel(std::to_string(rmseAzimuthBuffer.getBuffer().back().y));
	rmsePositionXValue->SetLabel(std::to_string(rmsePositionXBuffer.getBuffer().back().y));
	rmsePositionYValue->SetLabel(std::to_string(rmsePositionYBuffer.getBuffer().back().y));

	XYPlot* plot = new XYPlot();
	XYSimpleDataset* dataset = new XYSimpleDataset();

	dataset->AddSerie(new XYSerie(rmseAzimuthBuffer.getBuffer()));
	dataset->GetSerie(0)->SetName("RMSE azimuth");

	dataset->AddSerie(new XYSerie(rmsePositionXBuffer.getBuffer()));
	dataset->GetSerie(1)->SetName("RMSE position X");

	dataset->AddSerie(new XYSerie(rmsePositionYBuffer.getBuffer()));
	dataset->GetSerie(2)->SetName("RMSE position Y");

	dataset->SetRenderer(new XYLineRenderer());
	NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
	NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
	leftAxis->SetTitle(wxT("RMSE"));
	bottomAxis->SetTitle(wxT("Time [ms]"));
	if (rmseAzimuthBuffer.getBuffer().size() >= 100)
	{
		bottomAxis->SetFixedBounds(rmseAzimuthBuffer.getBuffer()[0].x, rmseAzimuthBuffer.getBuffer()[99].x);
	}
	Legend* legend = new Legend(wxTOP, wxRIGHT);
	plot->SetLegend(legend);
	plot->AddObjects(dataset, leftAxis, bottomAxis);

	Chart* chart = new Chart(plot, "RMSE");
	rmseChartPanel->SetChart(chart);
}
