#include "KalmanFilterSetupGui.h"

std::optional<kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth>> KalmanFilterSetupGui::getMatRazimuth()
{
    if (not isCallibrationDone)
    {
        wxMessageBox(wxT("ERR Kalibracja nie zosta³a jeszcze ukonczona"));
        return std::nullopt;
    }
    switch (movementModel)
    {
    case MovementModel::PEDESTRIAN:
        return matRAzimuthPedestrianAct;
    case MovementModel::RC_CAR:
        return matRAzimuthRcCarAct;
    case MovementModel::CAR:
        return matRAzimuthCarAct;
    default:
        wxMessageBox(wxT("Model ruchu pieszego NIE zosta³ wybrany - nie mo¿na rozpoczac filtracji"));
        return std::nullopt;
    }
}

std::optional<kf::Matrix<DIM_X_azimuth, DIM_X_azimuth>> KalmanFilterSetupGui::getMatQazimuth()
{
    if (not isCallibrationDone)
    {
        wxMessageBox(wxT("ERR Kalibracja nie zosta³a jeszcze ukonczona"));
        return std::nullopt;
    }
    switch (movementModel)
    {
    case MovementModel::PEDESTRIAN:
        return matQAzimuthPedestrianAct;
    case MovementModel::RC_CAR:
        return matQAzimuthRcCarAct;
    case MovementModel::CAR:
        return matQAzimuthCarAct;
    default:
        wxMessageBox(wxT("Model ruchu pieszego NIE zosta³ wybrany - nie mo¿na rozpoczac filtracji"));
        return std::nullopt;
    }
}

std::optional<kf::Matrix<DIM_Z, DIM_Z>> KalmanFilterSetupGui::getMatRacc()
{
    if (not isCallibrationDone)
    {
        wxMessageBox(wxT("ERR Kalibracja nie zosta³a jeszcze ukonczona"));
        return std::nullopt;
    }
    switch (movementModel)
    {
    case MovementModel::PEDESTRIAN:
        return matRAccPedestrianAct;
    case MovementModel::RC_CAR:
        return matRAccRcCarAct;
    case MovementModel::CAR:
        return matRAccCarAct;
    default:
        wxMessageBox(wxT("Model ruchu pieszego NIE zosta³ wybrany - nie mo¿na rozpoczac filtracji"));
        return std::nullopt;
    }
}

std::optional<kf::Matrix<DIM_X, DIM_X>> KalmanFilterSetupGui::getMatQacc()
{
    if (not isCallibrationDone)
    {
        wxMessageBox(wxT("ERR Kalibracja nie zosta³a jeszcze ukonczona"));
        return std::nullopt;
    }
    switch (movementModel)
    {
    case MovementModel::PEDESTRIAN:
        return matQAccPedestrianAct;
    case MovementModel::RC_CAR:
        return matQAccRcCarAct;
    case MovementModel::CAR:
        return matQAccCarAct;
    default:
        wxMessageBox(wxT("Model ruchu pieszego NIE zosta³ wybrany - nie mo¿na rozpoczac filtracji"));
        return std::nullopt;
    }
}

void KalmanFilterSetupGui::setupLeftPanel(wxPanel* mainPanel)
{
    wxBoxSizer* leftVerticalSizer = new wxBoxSizer(wxVERTICAL);
    //wxBoxSizer* measurementsTypeChooseVerticalSizer = new wxBoxSizer(wxVERTICAL);
    leftPanel = new wxPanel(mainPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_SIMPLE);

    chooseModel = new wxStaticText(leftPanel, wxID_ANY, "Ustawienie filtracji dla modeli ruchu");

    pedestrianModelButton = new wxRadioButton(leftPanel, wxID_ANY, wxT("Model ruchu pieszego"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP);
    rcCarModelButton = new wxRadioButton(leftPanel, wxID_ANY, wxT("Model ruchu samochodu RC"));
    carModelButton = new wxRadioButton(leftPanel, wxID_ANY, wxT("Model ruchu pojazdu samochodowego"));

    //realTimeMeasurementsButton = new wxRadioButton(leftPanel, wxID_ANY, wxT("Real time measurements"));
    //fromFileMeasurements = new wxRadioButton(leftPanel, wxID_ANY, wxT("From file measurements"));

    pedestrianModelButton->Bind(wxEVT_RADIOBUTTON, &KalmanFilterSetupGui::OnRadioButtonClicked, this);
    rcCarModelButton->Bind(wxEVT_RADIOBUTTON, &KalmanFilterSetupGui::OnRadioButtonClicked, this);
    carModelButton->Bind(wxEVT_RADIOBUTTON, &KalmanFilterSetupGui::OnRadioButtonClicked, this);

    //realTimeMeasurementsButton->Bind(wxEVT_RADIOBUTTON, &KalmanFilterSetupGui::OnRadioButtonRealTimeMeasurementsClicked, this);
    //fromFileMeasurements->Bind(wxEVT_RADIOBUTTON, &KalmanFilterSetupGui::OnRadioButtonRealTimeMeasurementsClicked, this);


    confirmCallibrationButton = new wxButton(leftPanel, wxID_ANY, wxT("ZatwierdŸ kalibracjê - rozpocznij filtracjê"));
    confirmCallibrationButton->SetMinSize(wxSize(300, 100));
    confirmCallibrationButton->SetBackgroundColour(wxColour(64, 255, 255));

    restartFiltrationAfterCallibrationButton = new wxButton(leftPanel, wxID_ANY, wxT("Restart filtracji po kalibracji"));
    restartFiltrationAfterCallibrationButton->SetMinSize(wxSize(300, 100));
    restartFiltrationAfterCallibrationButton->SetBackgroundColour(wxColour(64, 255, 255));

    //confirmMeasurementsTypeButton = new wxButton(leftPanel, wxID_ANY, wxT("ZatwierdŸ"));

    wxBoxSizer* mainSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* leftSizer = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* rightSizer = new wxBoxSizer(wxVERTICAL);

    wxStaticBoxSizer* staticBoxSizer = new wxStaticBoxSizer(wxVERTICAL, mainPanel, "Macierze filtracji");
    wxStaticBoxSizer* staticBoxSizerRadioButtons = new wxStaticBoxSizer(wxVERTICAL, mainPanel, "Model filtracji");


    stateVectorAzimuthName = new wxStaticText(leftPanel, wxID_ANY, "Wektor stanu X dla rotacji");
    createAndFillStateVectorRotationGrid();

    stateVectorPositionName = new wxStaticText(leftPanel, wxID_ANY, "Wektor stanu X dla pozycji");
    createAndFillStateVectorPositionGrid();

    matrixHAzimuthName = new wxStaticText(leftPanel, wxID_ANY, "Macierz H dla rotacji");
    createAndFillMatrixHRotationGrid();

    matrixHPositionName = new wxStaticText(leftPanel, wxID_ANY, "Macierz H dla pozycji");
    createAndFillMatrixHPositionGrid();

    leftSizer->Add(stateVectorAzimuthName, 0, wxALL | wxALIGN_CENTER, 5);
    leftSizer->Add(stateVectorAzimuthGrid, 0, wxALL | wxALIGN_CENTER, 5);

    leftSizer->AddSpacer(10);

    leftSizer->Add(matrixHAzimuthName, 0, wxALL | wxALIGN_CENTER, 5);
    leftSizer->Add(matrixHAzimuthGrid, 0, wxALL | wxALIGN_CENTER, 5);

    rightSizer->Add(stateVectorPositionName, 0, wxALL | wxALIGN_CENTER, 5);
    rightSizer->Add(stateVectorPositionGrid, 0, wxALL | wxALIGN_CENTER, 5);

    rightSizer->AddSpacer(10);

    rightSizer->Add(matrixHPositionName, 0, wxALL | wxALIGN_CENTER, 5);
    rightSizer->Add(matrixHPositionGrid, 0, wxALL | wxALIGN_CENTER, 5);

    mainSizer->Add(leftSizer, 0, wxALL, 5);
    mainSizer->Add(rightSizer, 0, wxALL, 5);

    //measurementsTypeChooseVerticalSizer->Add(realTimeMeasurementsButton, 0, wxALL, 5);
    //measurementsTypeChooseVerticalSizer->Add(fromFileMeasurements, 0, wxALL, 5);

    leftVerticalSizer->Add(chooseModel, 0, wxALL | wxALIGN_CENTER, 5);
    leftVerticalSizer->AddSpacer(10);
    staticBoxSizerRadioButtons->Add(pedestrianModelButton, 0, wxALL, 5);
    staticBoxSizerRadioButtons->Add(rcCarModelButton, 0, wxALL, 5);
    staticBoxSizerRadioButtons->Add(carModelButton, 0, wxALL, 5);

    leftVerticalSizer->Add(staticBoxSizerRadioButtons, 0, wxALIGN_CENTER | wxALL, 5);
    leftVerticalSizer->AddSpacer(10);
    staticBoxSizer->Add(mainSizer, 0, wxCENTER | wxALL, 10);

    leftVerticalSizer->Add(staticBoxSizer, 0, wxALIGN_CENTER | wxALL, 5);

    restartFiltrationAfterCallibrationButton->Enable(false);

    wxBoxSizer* buttonsSizer = new wxBoxSizer(wxHORIZONTAL);
    buttonsSizer->Add(confirmCallibrationButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    buttonsSizer->AddSpacer(20);
    buttonsSizer->Add(restartFiltrationAfterCallibrationButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

    //leftVerticalSizer->Add(confirmCallibrationButton, 0, wxALL | wxALIGN_CENTER, 5);
    leftVerticalSizer->Add(buttonsSizer, 0, wxALL | wxALIGN_CENTER, 5);

    restartFiltrationAfterCallibrationButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnRestartFiltration, this);
    confirmCallibrationButton->Bind(wxEVT_BUTTON, &KalmanFilterSetupGui::OnConfirmCallibration, this);

    leftPanel->SetSizer(leftVerticalSizer);
    //leftPanel->SetSizer(measurementsTypeChooseVerticalSizer);
    leftPanel->Layout();
}

void KalmanFilterSetupGui::setupRightPanel(wxPanel* mainPanel)
{
    rightVerticalSizer = new wxBoxSizer(wxVERTICAL);
    
    rightPanel = new wxPanel(mainPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_SIMPLE);
    rightPanel->SetBackgroundColour(*wxLIGHT_GREY);

    matrixRNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla k¹tu obrotu ruchu pieszego");
    matrixQNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla k¹tu obrotu ruchu pieszego");

    matrixRNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla po³o¿enia w ruchu pieszego");
    matrixQNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla po³o¿enia w ruchu pieszego");

    initMatrices();

    createGrids();

    fillMatRPedestrianAzimuth();
    fillMatQPedestrianAzimuth();
    fillMatRPedestrianAcc();
    fillMatQPedestrianAcc();

    setupSizer();
}

void KalmanFilterSetupGui::OnConfirmMeasType(wxCommandEvent& event)
{
    if (isCallibrationDone)
    {
        wxLogMessage(wxT("Nie mozna zmienic zrodla danych - filtracja w toku"));
        return;
    }
    if (measurementsType == MeasurementsType::NONE)
    {
        wxLogMessage(wxT("Nie wybrano zrodla danych"));
        return;
    }
    isMeasurementTypeChosen = true;

}

void KalmanFilterSetupGui::OnRadioButtonRealTimeMeasurementsClicked(wxCommandEvent& event)
{
    const int selectedId = event.GetId();
    if (selectedId == realTimeMeasurementsButton->GetId())
    {
        measurementsType = MeasurementsType::REAL_TIME;
    }
    else if (selectedId == fromFileMeasurements->GetId())
    {
        measurementsType = MeasurementsType::FROM_FILE;
    }
    else
    {
        wxLogMessage(wxT("Nieprawidlowe zrodlo pomiarow"));
    }
}

void KalmanFilterSetupGui::OnRadioButtonClicked(wxCommandEvent& event) 
{
    const int selectedId = event.GetId();
    if (isCallibrationDone)
    {
        wxLogMessage(wxT("Kalibracja zosta³a ju¿ zakoñczona - filtracja danych w toku"));
        return;
    }

    if (selectedId == pedestrianModelButton->GetId()) 
    {
        if (previousMovementModel == MovementModel::CAR)
        {
            handleKfMatricesForCar();
        }
        else if (previousMovementModel == MovementModel::RC_CAR)
        {
            handleKfMatricesForRcCar();
        }
        else
        {
            wxLogMessage(wxT("Nieprawidlowy poprzedni  model ruchu."));
        }
        previousMovementModel = MovementModel::PEDESTRIAN;

        rightVerticalSizer->Clear();
        destroyWidgets();

        matrixRNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla k¹tu obrotu ruchu pieszego");
        matrixQNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla k¹tu obrotu ruchu pieszego");

        matrixRNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla po³o¿enia w ruchu pieszego");
        matrixQNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla po³o¿enia w ruchu pieszego");

        createGrids();

        fillMatRPedestrianAzimuth();
        fillMatQPedestrianAzimuth();
        fillMatRPedestrianAcc();
        fillMatQPedestrianAcc();

        setupSizer();

        wxLogMessage(wxT("Wybrano model ruchu pieszego."));
    }
    else if (selectedId == rcCarModelButton->GetId()) 
    {
        if (previousMovementModel == MovementModel::CAR)
        {
            handleKfMatricesForCar();
        }
        else if (previousMovementModel == MovementModel::PEDESTRIAN)
        {
            handleKfMatricesForPedestrian();
        }
        else
        {
            wxLogMessage(wxT("Nieprawidlowy poprzedni  model ruchu."));
        }
        previousMovementModel = MovementModel::RC_CAR;

        rightVerticalSizer->Clear();
        destroyWidgets();

        matrixRNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla k¹tu obrotu ruchu pojazdu zdalnie sterowanego");
        matrixQNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla k¹tu obrotu ruchu pojazdu zdalnie sterowanego");

        matrixRNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla po³o¿enia w ruchu pojazdu zdalnie sterowanego");
        matrixQNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla po³o¿enia ruchu pojazdu zdalnie sterowanego");

        createGrids();

        fillMatRRcCarAzimuth();
        fillMatQRcCarAzimuth();
        fillMatRRcCarAcc();
        fillMatQRcCarAcc();

        setupSizer();

        wxLogMessage(wxT("Wybrano model ruchu pojazdu zdalnie sterowanego."));
    }
    else if (selectedId == carModelButton->GetId()) 
    {
        if (previousMovementModel == MovementModel::RC_CAR)
        {
            handleKfMatricesForRcCar();
        }
        else if (previousMovementModel == MovementModel::PEDESTRIAN)
        {
            handleKfMatricesForPedestrian();
        }
        else
        {
            wxLogMessage(wxT("Nieprawidlowy poprzedni  model ruchu."));
        }
        previousMovementModel = MovementModel::CAR;

        rightVerticalSizer->Clear();
        destroyWidgets();

        matrixRNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla k¹tu obrotu ruchu pojazdu samochodowego");
        matrixQNameForAzimuth = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla k¹tu obrotu ruchu pojazdu samochodowego");

        matrixRNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla po³o¿enia w ruchu pojazdu samochodowego");
        matrixQNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla po³o¿enia w ruchu pojazdu samochodowego");

        createGrids();

        fillMatRCarAzimuth();
        fillMatQCarAzimuth();
        fillMatRCarAcc();
        fillMatQCarAcc();

        setupSizer();

        wxLogMessage(wxT("Wybrano model ruchu pojazdu samochodowego."));
    }
    else
    {
        wxLogMessage(wxT("Nieprawid³owy model ruchu"));
    }
}

void KalmanFilterSetupGui::createWidgetsForAcc()
{
    matrixRNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz R dla po³o¿enia w ruchu pieszego");

    matrixRforAccFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
    matrixRforAccFilterPedestrianModel->HideRowLabels();
    matrixRforAccFilterPedestrianModel->HideColLabels();
    matrixRforAccFilterPedestrianModel->CreateGrid(3, 3);

    matrixRforAccFilterPedestrianModel->Show();

    matrixQNameForAcc = new wxStaticText(rightPanel, wxID_ANY, "Macierz Q dla po³o¿enia w ruchu pieszego");

    matrixQforAccFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
    matrixQforAccFilterPedestrianModel->HideRowLabels();
    matrixQforAccFilterPedestrianModel->HideColLabels();
    matrixQforAccFilterPedestrianModel->CreateGrid(6, 6);

    matrixQforAccFilterPedestrianModel->Show();
}

void KalmanFilterSetupGui::fillMatRPedestrianAzimuth()
{
    if (matrixRforAzimuthFilterPedestrianModel)
    {
        const int rows = matRAzimuthPedestrian.rows();
        const int cols = matRAzimuthPedestrian.cols();
        matrixRforAzimuthFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixRforAzimuthFilterPedestrianModel->SetCellValue
                    (row, col, wxString::Format("%.4f", matRAzimuthPedestrian(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatQPedestrianAzimuth()
{
    if (matrixQforAzimuthFilterPedestrianModel)
    {
        int rows = matQAzimuthPedestrian.rows();
        int cols = matQAzimuthPedestrian.cols();
        matrixQforAzimuthFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixQforAzimuthFilterPedestrianModel->SetCellValue(row, col, wxString::Format("%.6f", matQAzimuthPedestrian(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatRPedestrianAcc()
{
    if (matrixRforAccFilterPedestrianModel)
    {
        const int rows = matRAccPedestrian.rows();
        const int cols = matRAccPedestrian.cols();
        matrixRforAccFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixRforAccFilterPedestrianModel->SetCellValue
                (row, col, wxString::Format("%.4f", matRAccPedestrian(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatQPedestrianAcc()
{
    if (matrixQforAccFilterPedestrianModel)
    {
        int rows = matQAccPedestrian.rows();
        int cols = matQAccPedestrian.cols();
        matrixQforAccFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixQforAccFilterPedestrianModel->SetCellValue(row, col, wxString::Format("%.6f", matQAccPedestrian(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatRRcCarAzimuth()
{
    if (matrixRforAzimuthFilterPedestrianModel)
    {
        const int rows = matRAzimuthRcCar.rows();
        const int cols = matRAzimuthRcCar.cols();
        matrixRforAzimuthFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixRforAzimuthFilterPedestrianModel->SetCellValue
                (row, col, wxString::Format("%.4f", matRAzimuthRcCar(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatQRcCarAzimuth()
{
    if (matrixQforAzimuthFilterPedestrianModel)
    {
        int rows = matQAzimuthRcCar.rows();
        int cols = matQAzimuthRcCar.cols();
        matrixQforAzimuthFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixQforAzimuthFilterPedestrianModel->SetCellValue(row, col, wxString::Format("%.6f", matQAzimuthRcCar(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatRRcCarAcc()
{
    if (matrixRforAccFilterPedestrianModel)
    {
        const int rows = matRAccRcCar.rows();
        const int cols = matRAccRcCar.cols();
        matrixRforAccFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixRforAccFilterPedestrianModel->SetCellValue
                (row, col, wxString::Format("%.4f", matRAccRcCar(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatQRcCarAcc()
{
    if (matrixQforAccFilterPedestrianModel)
    {
        int rows = matQAccRcCar.rows();
        int cols = matQAccRcCar.cols();
        matrixQforAccFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixQforAccFilterPedestrianModel->SetCellValue(row, col, wxString::Format("%.6f", matQAccRcCar(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatRCarAzimuth()
{
    if (matrixRforAzimuthFilterPedestrianModel)
    {
        const int rows = matRAzimuthCar.rows();
        const int cols = matRAzimuthCar.cols();
        matrixRforAzimuthFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixRforAzimuthFilterPedestrianModel->SetCellValue
                (row, col, wxString::Format("%.4f", matRAzimuthCar(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatQCarAzimuth()
{
    if (matrixQforAzimuthFilterPedestrianModel)
    {
        int rows = matQAzimuthCar.rows();
        int cols = matQAzimuthCar.cols();
        matrixQforAzimuthFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixQforAzimuthFilterPedestrianModel->SetCellValue(row, col, wxString::Format("%.6f", matQAzimuthCar(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatRCarAcc()
{
    if (matrixRforAccFilterPedestrianModel)
    {
        const int rows = matRAccCar.rows();
        const int cols = matRAccCar.cols();
        matrixRforAccFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixRforAccFilterPedestrianModel->SetCellValue
                (row, col, wxString::Format("%.4f", matRAccCar(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::fillMatQCarAcc()
{
    if (matrixQforAccFilterPedestrianModel)
    {
        int rows = matQAccCar.rows();
        int cols = matQAccCar.cols();
        matrixQforAccFilterPedestrianModel->CreateGrid(rows, cols);

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                matrixQforAccFilterPedestrianModel->SetCellValue(row, col, wxString::Format("%.6f", matQAccCar(row, col)));
            }
        }
    }
}

void KalmanFilterSetupGui::destroyWidgets()
{
    if (matrixRNameForAzimuth)
    {
        matrixRNameForAzimuth->Destroy();
    }
    if (matrixRforAzimuthFilterPedestrianModel)
    {
        matrixRforAzimuthFilterPedestrianModel->Destroy();
    }
    if (matrixQNameForAzimuth)
    {
        matrixQNameForAzimuth->Destroy();
    }
    if (matrixQforAzimuthFilterPedestrianModel)
    {
        matrixQforAzimuthFilterPedestrianModel->Destroy();
    }
    if (matrixRNameForAcc)
    {
        matrixRNameForAcc->Destroy();
    }
    if (matrixRforAccFilterPedestrianModel)
    {
        matrixRforAccFilterPedestrianModel->Destroy();
    }
    if (matrixQNameForAcc)
    {
        matrixQNameForAcc->Destroy();
    }
    if (matrixQforAccFilterPedestrianModel)
    {
        matrixQforAccFilterPedestrianModel->Destroy();
    }
}

void KalmanFilterSetupGui::createGrids()
{
    //if (matrixRforAzimuthFilterPedestrianModel == nullptr)
    {
        matrixRforAzimuthFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
        matrixRforAzimuthFilterPedestrianModel->HideRowLabels();
        matrixRforAzimuthFilterPedestrianModel->HideColLabels();
    }
    //if (matrixQforAzimuthFilterPedestrianModel == nullptr)
    {
        matrixQforAzimuthFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
        matrixQforAzimuthFilterPedestrianModel->HideRowLabels();
        matrixQforAzimuthFilterPedestrianModel->HideColLabels();
    }
    //if (matrixRforAccFilterPedestrianModel == nullptr)
    {
        matrixRforAccFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
        matrixRforAccFilterPedestrianModel->HideRowLabels();
        matrixRforAccFilterPedestrianModel->HideColLabels();
    }
    //if (matrixQforAccFilterPedestrianModel == nullptr)
    {
        matrixQforAccFilterPedestrianModel = new wxGrid(rightPanel, wxID_ANY);
        matrixQforAccFilterPedestrianModel->HideRowLabels();
        matrixQforAccFilterPedestrianModel->HideColLabels();
    }
}

void KalmanFilterSetupGui::setupSizer()
{
    rightVerticalSizer->Add(matrixRNameForAzimuth, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->Add(matrixRforAzimuthFilterPedestrianModel, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->AddSpacer(15);
    rightVerticalSizer->Add(matrixQNameForAzimuth, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->Add(matrixQforAzimuthFilterPedestrianModel, 0, wxALL | wxALIGN_CENTER, 5);

    rightVerticalSizer->Add(matrixRNameForAcc, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->Add(matrixRforAccFilterPedestrianModel, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->AddSpacer(15);
    rightVerticalSizer->Add(matrixQNameForAcc, 0, wxALL | wxALIGN_CENTER, 5);
    rightVerticalSizer->Add(matrixQforAccFilterPedestrianModel, 0, wxALL | wxALIGN_CENTER, 5);

    rightPanel->SetSizer(rightVerticalSizer);
    rightPanel->Layout();
}

void KalmanFilterSetupGui::createAndFillStateVectorRotationGrid()
{
    stateVectorAzimuthGrid = new wxGrid(leftPanel, wxID_ANY);
    stateVectorAzimuthGrid->HideRowLabels();
    stateVectorAzimuthGrid->HideColLabels();
    stateVectorAzimuthGrid->CreateGrid(2, 1);

    attr = new wxGridCellAttr();
    attr->SetReadOnly(true);


    stateVectorAzimuthGrid->SetCellValue(0, 0, "Z angular velocity");
    stateVectorAzimuthGrid->SetCellValue(1, 0, "Z rotation");
    stateVectorAzimuthGrid->SetColSize(0, 100);
}

void KalmanFilterSetupGui::createAndFillStateVectorPositionGrid()
{
    stateVectorPositionGrid = new wxGrid(leftPanel, wxID_ANY);
    stateVectorPositionGrid->HideRowLabels();
    stateVectorPositionGrid->HideColLabels();
    stateVectorPositionGrid->CreateGrid(6, 1);

    stateVectorPositionGrid->SetCellValue(0, 0, "x acceleration");
    stateVectorPositionGrid->SetCellValue(1, 0, "x velocity");
    stateVectorPositionGrid->SetCellValue(2, 0, "x distance");
    stateVectorPositionGrid->SetCellValue(3, 0, "y acceleration");
    stateVectorPositionGrid->SetCellValue(4, 0, "y velocity");
    stateVectorPositionGrid->SetCellValue(5, 0, "y distance");

    stateVectorPositionGrid->SetColSize(0, 100);
}

void KalmanFilterSetupGui::createAndFillMatrixHRotationGrid()
{
    matrixHAzimuthGrid = new wxGrid(leftPanel, wxID_ANY);
    matrixHAzimuthGrid->HideRowLabels();
    matrixHAzimuthGrid->HideColLabels();
    matrixHAzimuthGrid->SetDefaultColSize(25);

    kf::Matrix<DIM_Z_azimuth, DIM_X_azimuth> matH;
    matH <<
        0, 0, 0,
        1, 0, 0;

    matrixHAzimuthGrid->CreateGrid(matH.rows(), matH.cols());

    for (int row = 0; row < matH.rows(); ++row) {
        for (int col = 0; col < matH.cols(); ++col) {
            matrixHAzimuthGrid->SetCellValue(row, col, wxString::Format("%g", matH(row, col)));
        }
    }
}

void KalmanFilterSetupGui::createAndFillMatrixHPositionGrid()
{
    matrixHPositionGrid = new wxGrid(leftPanel, wxID_ANY);
    matrixHPositionGrid->HideRowLabels();
    matrixHPositionGrid->HideColLabels();
    matrixHPositionGrid->SetDefaultColSize(25);

    kf::Matrix<DIM_Z, DIM_X> matH;
    matH <<
        1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F;

    matrixHPositionGrid->CreateGrid(matH.rows(), matH.cols());

    for (int row = 0; row < matH.rows(); ++row) {
        for (int col = 0; col < matH.cols(); ++col) {
            matrixHPositionGrid->SetCellValue(row, col, wxString::Format("%g", matH(row, col)));
        }
    }
}

void KalmanFilterSetupGui::handleKfMatricesForPedestrian()
{
    const bool matRAzimuthPed = handleMatRazimuth(matRAzimuthPedestrian);
    const bool matQAzimuthPed = handleMatQazimuth(matQAzimuthPedestrian);
    const bool matRAccPed = handleMatRAcc(matRAccPedestrian);
    const bool matQAccPed = handleMatQAcc(matQAccPedestrian);
}

void KalmanFilterSetupGui::handleKfMatricesForRcCar()
{
    const bool matRAzimuthPed = handleMatRazimuth(matRAzimuthRcCar);
    const bool matQAzimuthPed = handleMatQazimuth(matQAzimuthRcCar);
    const bool matRAccPed = handleMatRAcc(matRAccRcCar);
    const bool matQAccPed = handleMatQAcc(matQAccRcCar);
}

void KalmanFilterSetupGui::handleKfMatricesForCar()
{
    const bool matRAzimuthPed = handleMatRazimuth(matRAzimuthCar);
    const bool matQAzimuthPed = handleMatQazimuth(matQAzimuthCar);
    const bool matRAccPed = handleMatRAcc(matRAccCar);
    const bool matQAccPed = handleMatQAcc(matQAccCar);
}


bool KalmanFilterSetupGui::handleMatRazimuth(kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth>& matRAzimuth)
{
    int numRows = matrixRforAzimuthFilterPedestrianModel->GetNumberRows();
    int numCols = matrixRforAzimuthFilterPedestrianModel->GetNumberCols();
    if (numRows != DIM_Z_azimuth || numCols != DIM_Z_azimuth)
    {
        wxLogMessage(wxT("Nieprawidlowy rozmiar macierzy R azimuth"));
        return false;
    }

    for (int row = 0; row < numRows; ++row) {
        for (int col = 0; col < numCols; ++col) {
            wxString valueStr = matrixRforAzimuthFilterPedestrianModel->GetCellValue(row, col);
            double value;
            valueStr.ToDouble(&value);
            matRAzimuth(row, col) = value;
        }
    }
    return true;
}

bool KalmanFilterSetupGui::handleMatQazimuth(kf::Matrix<DIM_X_azimuth, DIM_X_azimuth>& matQAzimuth)
{
    int numRows1 = matrixQforAzimuthFilterPedestrianModel->GetNumberRows();
    int numCols1 = matrixQforAzimuthFilterPedestrianModel->GetNumberCols();
    if (numRows1 != DIM_X_azimuth || numCols1 != DIM_X_azimuth)
    {
        wxLogMessage(wxT("Nieprawidlowy rozmiar macierzy Q azimuth"));
        return false;
    }

    for (int row = 0; row < numRows1; ++row) {
        for (int col = 0; col < numCols1; ++col) {
            wxString valueStr = matrixQforAzimuthFilterPedestrianModel->GetCellValue(row, col);
            double value;
            valueStr.ToDouble(&value);
            matQAzimuth(row, col) = value;
        }
    }
    return true;
}

bool KalmanFilterSetupGui::handleMatRAcc(kf::Matrix<DIM_Z, DIM_Z>& matRAcc)
{
    int numRows2 = matrixRforAccFilterPedestrianModel->GetNumberRows();
    int numCols2 = matrixRforAccFilterPedestrianModel->GetNumberCols();
    if (numRows2 != DIM_Z || numCols2 != DIM_Z)
    {
        wxLogMessage(wxT("Nieprawidlowy rozmiar macierzy R acc"));
        return false;
    }

    for (int row = 0; row < numRows2; ++row) {
        for (int col = 0; col < numCols2; ++col) {
            wxString valueStr = matrixRforAccFilterPedestrianModel->GetCellValue(row, col);
            double value;
            valueStr.ToDouble(&value);
            matRAcc(row, col) = value;
        }
    }
    return true;
}

bool KalmanFilterSetupGui::handleMatQAcc(kf::Matrix<DIM_X, DIM_X>& matQAcc)
{
    int numRows3 = matrixQforAccFilterPedestrianModel->GetNumberRows();
    int numCols3 = matrixQforAccFilterPedestrianModel->GetNumberCols();
    if (numRows3 != DIM_X || numCols3 != DIM_X)
    {
        wxLogMessage(wxT("Nieprawidlowy rozmiar macierzy Q acc"));
        return false;
    }

    for (int row = 0; row < numRows3; ++row) {
        for (int col = 0; col < numCols3; ++col) {
            wxString valueStr = matrixQforAccFilterPedestrianModel->GetCellValue(row, col);
            double value;
            valueStr.ToDouble(&value);
            matQAcc(row, col) = value;
        }
    }
    return true;
}

void KalmanFilterSetupGui::initMatrices()
{
    initPedestrianModelMatrices();
    initRcCarModelMatrices();
    initCarModelMatrices();
}

void KalmanFilterSetupGui::initPedestrianModelMatrices()
{
    matRAzimuthPedestrian << 
        0.0001F, 0, 0,
        0, 0.1F, 0,
        0, 0, 0.1F;

    double process_variance = 0.02F;
    double coefficient = 5.0F;
    matQAzimuthPedestrian <<
        coefficient, 0.0F,
        0.0F, coefficient;


    matQAzimuthPedestrian *= process_variance;

    matRAccPedestrian <<
        0.03F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.03F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.03F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.03F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F;

    double processVarianceAcc = 1.0F;
    double sCoefficient = 0.00000001F;


    matQAccPedestrian <<
        sCoefficient/32, 0, 0, 0, 0, 0,
        0, sCoefficient/32, 0, 0, 0, 0,
        0, 0, sCoefficient/64, 0, 0, 0,
        0, 0, 0, sCoefficient/64, 0, 0,
        0, 0, 0, 0, 10.0F, 0,
        0, 0, 0, 0, 0, 10.0F;

    matQAccPedestrian *= processVarianceAcc;
}

void KalmanFilterSetupGui::initRcCarModelMatrices()
{
    matRAzimuthRcCar << 0.0001F, 0, 0,
                        0, 0.1F, 0,
                        0, 0, 0.1F;

    double process_variance = 0.02F;
    double coefficient = 0.1F;
    matQAzimuthRcCar <<
        coefficient, 0.0F,
        0.0F, coefficient;


    matQAzimuthRcCar *= process_variance;

    matRAccRcCar <<
        0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F;

    double processVarianceAcc = 0.002F;
    double sCoefficient = 100.0F;
    sCoefficient = sCoefficient / 100000.0F;

    matQAccRcCar << 
        sCoefficient, 0, 0, 0, 0, 0,
        0, sCoefficient, 0, 0, 0, 0,
        0, 0, sCoefficient, 0, 0, 0,
        0, 0, 0, sCoefficient, 0, 0,
        0, 0, 0, 0, sCoefficient, 0,
        0, 0, 0, 0, 0, sCoefficient;


    matQAccRcCar *= processVarianceAcc;
}

void KalmanFilterSetupGui::initCarModelMatrices()
{
    double rCoefficientAzimut{ 0.00054F };
    matRAzimuthCar << 
        rCoefficientAzimut / 10000.0F, 0, 0,
        0, rCoefficientAzimut / 1000.0F, 0,
        0, 0, rCoefficientAzimut / 10.0F;

    double process_variance = 0.02F;
    double coefficient = 6.25F;
    matQAzimuthCar <<
        coefficient, 0.0F,
        0.0F, coefficient;

    matQAzimuthCar *= process_variance;

    double rCoefficient{ 130.0F };

    matRAccCar <<
        rCoefficient/ 0.8F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, rCoefficient /0.8F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 64.0F * rCoefficient * 1.4F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 64.0F * rCoefficient * 1.4F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F;

    //double processVarianceAcc = 110.02F;
    //double sCoefficient = 100.0F;
    //sCoefficient = sCoefficient / 10.0F;

    double processVarianceAcc = 0.3F;
    double sCoefficient = 0.005F;

    matQAccCar << 
        sCoefficient / 0.1F, 0, 0, 0, 0, 0,
        0, sCoefficient / 0.1F, 0, 0, 0, 0,
        0, 0, sCoefficient * 1.0F, 0, 0, 0,
        0, 0, 0, sCoefficient * 1.0F, 0, 0,
        0, 0, 0, 0, 0.0F, 0,
        0, 0, 0, 0, 0, 0.0F;

    matQAccCar *= processVarianceAcc;
}

void KalmanFilterSetupGui::OnConfirmCallibration(wxCommandEvent& event)
{
    restartFiltrationAfterCallibrationButton->Enable(true);
    //if (not magnChartGui.checkIfMagnCalibrationDone())
    //{
    //    wxMessageBox(wxT("Magnetometer callibration was not done!"));
    //    return;
    //}
    if (isCallibrationDone)
    {
        wxMessageBox(wxT("Filtracja danych w toku - nie mozna zmienic wybranego modelu ruchu"));
        return;
    }

    if (pedestrianModelButton->GetValue())
    {
        isCallibrationDone = true;
        filterReceivedDataProcessingTimer.Start(70);
        filterReceivedGpsProcessingTimer.Start(1000);
        isRestartFiltrationNeeded = true;
        movementModel = MovementModel::PEDESTRIAN;

        handleKfMatricesForPedestrian();
        assignActiveMatrices();

        wxMessageBox(wxT("Model ruchu pieszego zosta³ wybrany - rozpoczêto filtracjê"));
    }
    else if (rcCarModelButton->GetValue())
    {
        isCallibrationDone = true;
        filterReceivedDataProcessingTimer.Start(70);
        filterReceivedGpsProcessingTimer.Start(1000);
        isRestartFiltrationNeeded = true;
        movementModel = MovementModel::RC_CAR;

        handleKfMatricesForRcCar();
        assignActiveMatrices();

        wxMessageBox(wxT("Model ruchu RC samochodu zosta³ wybrany - rozpoczêto filtracjê"));
    }
    else if (carModelButton->GetValue())
    {
        isCallibrationDone = true;
        filterReceivedDataProcessingTimer.Start(70);
        filterReceivedGpsProcessingTimer.Start(1000);
        isRestartFiltrationNeeded = true;
        movementModel = MovementModel::CAR;

        handleKfMatricesForCar();
        assignActiveMatrices();

        wxMessageBox(wxT("Model ruchu pojazd samochodowy zosta³ wybrany - rozpoczêto filtracjê"));
    }
    else
    {
        movementModel = MovementModel::NONE;
        wxMessageBox(wxT("¯aden z przycisków nie zosta³ zaznaczony"));
    }
}

void KalmanFilterSetupGui::OnRestartFiltration(wxCommandEvent& event)
{
    

    if (pedestrianModelButton->GetValue())
    {
        isFiltrationRestarted = true;
        movementModel = MovementModel::PEDESTRIAN;

        handleKfMatricesForPedestrian();
        assignActiveMatrices();

        wxMessageBox(wxT("Model ruchu pieszego zosta³ wybrany - rozpoczêto filtracjê"));
    }
    else if (rcCarModelButton->GetValue())
    {
        isFiltrationRestarted = true;
        movementModel = MovementModel::RC_CAR;

        handleKfMatricesForRcCar();
        assignActiveMatrices();

        wxMessageBox(wxT("Model ruchu RC samochodu zosta³ wybrany - rozpoczêto filtracjê"));
    }
    else if (carModelButton->GetValue())
    {
        isFiltrationRestarted = true;
        movementModel = MovementModel::CAR;

        handleKfMatricesForCar();
        assignActiveMatrices();

        wxMessageBox(wxT("Model ruchu pojazd samochodowy zosta³ wybrany - rozpoczêto filtracjê"));
    }
    else
    {
        movementModel = MovementModel::NONE;
        wxMessageBox(wxT("¯aden z przycisków nie zosta³ zaznaczony"));
    }
}

void KalmanFilterSetupGui::assignActiveMatrices()
{
    matRAccRcCarAct = matRAccRcCar;
    matQAccRcCarAct = matQAccRcCar;
    matRAccPedestrianAct = matRAccPedestrian;
    matQAccPedestrianAct = matQAccPedestrian;
    matRAccCarAct = matRAccCar;
    matQAccCarAct = matQAccCar;

    matRAzimuthRcCarAct = matRAzimuthRcCar;
    matQAzimuthRcCarAct = matQAzimuthRcCar;
    matRAzimuthPedestrianAct = matRAzimuthPedestrian;
    matQAzimuthPedestrianAct = matQAzimuthPedestrian;
    matRAzimuthCarAct = matRAzimuthCar;
    matQAzimuthCarAct = matQAzimuthCar;
}