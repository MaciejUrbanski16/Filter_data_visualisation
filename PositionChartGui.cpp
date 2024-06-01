#include "PositionChartGui.h"

void PositionChartGui::setup(wxNotebook* m_notebook)
{
	positionChartPanel = new wxChartPanel(m_notebook);
	m_notebook->AddPage(positionChartPanel, "Pos chart");
}

void PositionChartGui::updateChart(PlotElementsBuffer& rawPositionBuffer, const double xDistance, const double yDistance, const double timeMs)
{
    currentXPos = currentXPos + xDistance;
    currentYPos = currentYPos + yDistance;
    rawPositionBuffer.AddElement(wxRealPoint(currentXPos, currentYPos));

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(rawPositionBuffer.getBuffer()));
    dataset->SetRenderer(new XYLineRenderer());
    dataset->GetSerie(0)->SetName("Raw position");
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Y position [m]"));
    bottomAxis->SetTitle(wxT("X position [m]"));
    Legend* legend = new Legend(wxTOP, wxLEFT);
    plot->SetLegend(legend);
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    Chart* chart = new Chart(plot, "Position");

    positionChartPanel->SetChart(chart);
}