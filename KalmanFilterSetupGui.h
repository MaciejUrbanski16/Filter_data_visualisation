#pragma once
#include <wx/wx.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/grid.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/splitter.h>
#include <wx/spinctrl.h> 
#include <wx/listctrl.h>

#include "kalman_filter/kalman_filter.h"

#include <optional>
#include "MagnChartGui.h"

static constexpr size_t DIM_X_azimuth{ 2 };
static constexpr size_t DIM_Z_azimuth{ 3 };
static constexpr size_t DIM_X_gyro{ 6 };
static constexpr size_t DIM_Z_gyro{ 3 };
static constexpr size_t DIM_X{ 6 };
static constexpr size_t DIM_Z{ 6 };

enum class MovementModel
{
    PEDESTRIAN = 0,
    RC_CAR,
    CAR,
    NONE
};

enum class MeasurementsType
{
    NONE = 0,
    REAL_TIME,
    FROM_FILE
};

class KalmanFilterSetupGui
{
public:
    KalmanFilterSetupGui(wxTimer& filterReceivedDataProcessingTimer, wxTimer& filterReceivedGpsProcessingTimer,
        MagnChartGui& magnChartGui)
        : 
        filterReceivedDataProcessingTimer(filterReceivedDataProcessingTimer),
        filterReceivedGpsProcessingTimer(filterReceivedGpsProcessingTimer),
        magnChartGui(magnChartGui)
        {}

    std::optional<kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth>> getMatRazimuth();
    std::optional<kf::Matrix<DIM_X_azimuth, DIM_X_azimuth>> getMatQazimuth();
    std::optional<kf::Matrix<DIM_Z, DIM_Z>> getMatRacc();
    std::optional<kf::Matrix<DIM_X, DIM_X>> getMatQacc();

	void setup(wxNotebook* m_notebook)
	{ 
        wxBoxSizer* mainVerticalSizer = new wxBoxSizer(wxHORIZONTAL);
        wxPanel* mainPanel = new wxPanel(m_notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_SIMPLE);

        setupLeftPanel(mainPanel);
        setupRightPanel(mainPanel);

        mainVerticalSizer->Add(leftPanel, 1, wxEXPAND | wxALL, 5);
        mainVerticalSizer->Add(rightPanel, 1, wxEXPAND | wxALL, 5);

        mainPanel->SetSizer(mainVerticalSizer);
        mainPanel->Layout();

        m_notebook->AddPage(mainPanel, "KF setup");
	}

    bool getIsCallibrationDone() const
    {
        return isCallibrationDone;
    }

    bool getIsRestartFiltrationNeeded() const
    {
        return isRestartFiltrationNeeded;
    }
    void setIsRestartFiltrationNeeded(bool value)
    {
        isRestartFiltrationNeeded = value;
    }

    bool getIsFiltrationRestarted() const
    {
        return isFiltrationRestarted;
    }
    void setIsFiltrationRestarted(bool value)
    {
        isFiltrationRestarted = value;
    }

    MovementModel getMovementModel() const
    {
        return movementModel;
    }

private:
    void setupLeftPanel(wxPanel* mainPanel);
    void setupRightPanel(wxPanel* mainPanel);

    void OnConfirmCallibration(wxCommandEvent& event);
    void OnRestartFiltration(wxCommandEvent& event);

    void OnRadioButtonClicked(wxCommandEvent& event);
    void OnRadioButtonRealTimeMeasurementsClicked(wxCommandEvent& event);
    void OnConfirmMeasType(wxCommandEvent& event);

    wxTimer& filterReceivedDataProcessingTimer;
    wxTimer& filterReceivedGpsProcessingTimer;
    MagnChartGui& magnChartGui;

    wxPanel* kalmanParamsSetupPanel = nullptr;
	wxButton* confirmCallibrationButton = nullptr;
    wxButton* confirmMeasurementsTypeButton = nullptr;
    bool isMeasurementTypeChosen{ false };

    wxButton* restartFiltrationAfterCallibrationButton = nullptr;
    wxBoxSizer* sizer = nullptr;
    wxSplitterWindow* azimuthPanelSplitter = nullptr;

    wxPanel* leftPanel = nullptr;
    wxPanel* rightPanel = nullptr;

    wxRadioButton* pedestrianModelButton = nullptr;
    wxRadioButton* rcCarModelButton = nullptr;
    wxRadioButton* carModelButton = nullptr;

    wxRadioButton* realTimeMeasurementsButton = nullptr;
    wxRadioButton* fromFileMeasurements = nullptr;;

    wxBoxSizer* rightVerticalSizer = nullptr;

    wxStaticText* matrixRNameForAzimuth = nullptr;
    wxStaticText* matrixQNameForAzimuth = nullptr;
    wxStaticText* matrixRNameForAcc = nullptr;
    wxStaticText* matrixQNameForAcc = nullptr;

    wxGrid* matrixRforAzimuthFilterPedestrianModel = nullptr;
    wxGrid* matrixRforAccFilterPedestrianModel = nullptr;
    wxGrid* matrixQforAzimuthFilterPedestrianModel = nullptr;
    wxGrid* matrixQforAccFilterPedestrianModel = nullptr;

   // wxGrid* matrixRforAzimuthFilterCarModel = nullptr;
   // wxGrid* matrixRforAzimuthFilterRCcarModel = nullptr;

    void createAndFillStateVectorRotationGrid();
    void createAndFillStateVectorPositionGrid();

    void createAndFillMatrixHRotationGrid();
    void createAndFillMatrixHPositionGrid();

    wxStaticText* chooseModel = nullptr;

    wxStaticText* stateVectorAzimuthName = nullptr;
    wxGrid* stateVectorAzimuthGrid = nullptr;
    wxGridCellAttr* attr = nullptr;

    wxStaticText* stateVectorPositionName = nullptr;
    wxGrid* stateVectorPositionGrid = nullptr;

    wxStaticText* matrixHAzimuthName = nullptr;
    wxGrid* matrixHAzimuthGrid = nullptr;

    wxStaticText* matrixHPositionName = nullptr;
    wxGrid* matrixHPositionGrid = nullptr;

    wxTextCtrl* textCtrl = nullptr;

    MovementModel movementModel{ MovementModel::NONE };
    MovementModel previousMovementModel{ MovementModel::PEDESTRIAN };
    bool isCallibrationDone = false;
    bool isRestartFiltrationNeeded = false;
    bool isFiltrationRestarted{ false };

    MeasurementsType measurementsType{ MeasurementsType::NONE };


    //PoC
    void destroyWidgets();
    void createGrids();
    void setupSizer();

    void initMatrices();
    void initPedestrianModelMatrices();
    void initRcCarModelMatrices();
    void initCarModelMatrices();

    void createWidgetsForAcc();

    void fillMatRPedestrianAzimuth();
    void fillMatQPedestrianAzimuth();
    void fillMatRPedestrianAcc();
    void fillMatQPedestrianAcc();

    void fillMatRCarAzimuth();
    void fillMatQCarAzimuth();
    void fillMatRCarAcc();
    void fillMatQCarAcc();

    void fillMatRRcCarAzimuth();
    void fillMatQRcCarAzimuth();
    void fillMatRRcCarAcc();
    void fillMatQRcCarAcc();

    //TEMPORARY MATRICES
    kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth> matRAzimuthPedestrian;
    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> matQAzimuthPedestrian;
    kf::Matrix<DIM_Z, DIM_Z> matRAccPedestrian;
    kf::Matrix<DIM_X, DIM_X> matQAccPedestrian;

    kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth> matRAzimuthRcCar;
    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> matQAzimuthRcCar;
    kf::Matrix<DIM_Z, DIM_Z> matRAccRcCar;
    kf::Matrix<DIM_X, DIM_X> matQAccRcCar;

    kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth> matRAzimuthCar;
    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> matQAzimuthCar;
    kf::Matrix<DIM_Z, DIM_Z> matRAccCar;
    kf::Matrix<DIM_X, DIM_X> matQAccCar;

    //ACTIVE MATRICES
    kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth> matRAzimuthPedestrianAct;
    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> matQAzimuthPedestrianAct;
    kf::Matrix<DIM_Z, DIM_Z> matRAccPedestrianAct;
    kf::Matrix<DIM_X, DIM_X> matQAccPedestrianAct;

    kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth> matRAzimuthRcCarAct;
    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> matQAzimuthRcCarAct;
    kf::Matrix<DIM_Z, DIM_Z> matRAccRcCarAct;
    kf::Matrix<DIM_X, DIM_X> matQAccRcCarAct;

    kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth> matRAzimuthCarAct;
    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> matQAzimuthCarAct;
    kf::Matrix<DIM_Z, DIM_Z> matRAccCarAct;
    kf::Matrix<DIM_X, DIM_X> matQAccCarAct;

    void handleKfMatricesForPedestrian();
    void handleKfMatricesForRcCar();
    void handleKfMatricesForCar();

    bool handleMatRazimuth(kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth>& matRAzimuth);
    bool handleMatQazimuth(kf::Matrix<DIM_X_azimuth, DIM_X_azimuth>& matQAzimuth);
    bool handleMatRAcc(kf::Matrix<DIM_Z, DIM_Z>& matRAcc);
    bool handleMatQAcc(kf::Matrix<DIM_X, DIM_X>& matQAcc);

    void assignActiveMatrices();
};

