#include "MainWindow.h"


wxThread::ExitCode SensorDataReceptionThread::Entry()
{
    boost::asio::io_context io;

    Server server(io, 8081, appLogger, m_parent);
    io.run();

    const std::string threadFinished{ "SensorDataReceptionThread Thread finished successfully.\n" };
    appLogger.logSerialCommStartThread(threadFinished);

    return NULL;
}

////////////////////


wxThread::ExitCode SensorDataComReceptionThread::Entry()
{
    boost::asio::io_context io;
    std::string com = "COM7";

    SerialComSensorDataReceiver serialComm(io, com, m_parent);
    io.run();

    const std::string threadFinished{ "SensorDataComReceptionThread Thread finished successfully.\n" };
    appLogger.logSerialCommStartThread(threadFinished);

    return NULL;
}


MyWindow::MyWindow(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(1080, 680))
{
    Centre();
    prepareGui();

    createSensorDataReceptionThread();
}


void MyWindow::OnParityChoice(wxCommandEvent& event)
{
    int selection = event.GetInt(); // Get the selected index
    wxLogMessage("Selected parity: %s", event.GetString().c_str());
}

void MyWindow::OnStartReceptionClick(wxCommandEvent& event) 
{
    wxLogMessage("Data reception starts - new thread will be created!");
}

void MyWindow::OnResetAccChart(wxCommandEvent& event)
{
    wxLogMessage("Reset acceleration chart!");
    xAccPoints.clear();
    yAccPoints.clear();
    zAccPoints.clear();
    timeNewAccPoint = 0;
}

void MyWindow::OnSubmitAccAdjustments(wxCommandEvent& event)
{
    const int xAccCtrlValue = spinCtrlXacc->GetValue();
    const int yAccCtrlValue = spinCtrlYacc->GetValue();
    const int zAccCtrlValue = spinCtrlZacc->GetValue();

    rawGrawity = zAccCtrlValue;
    xBias = xAccCtrlValue;
    yBias = yAccCtrlValue;
    wxLogMessage("New acc adj X:%d Y:%d Z:%d!", xAccCtrlValue, yAccCtrlValue, zAccCtrlValue);
}

void MyWindow::OnSpinXAccIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlXaccMultiplicator->GetValue();
    spinCtrlXacc->SetIncrement(newIncrement);
}

void MyWindow::OnSpinXAccUpdate(wxSpinEvent& event)
{
    const int xAccCtrlValue = spinCtrlXacc->GetValue();
    xBias = xAccCtrlValue;
}

void MyWindow::OnSpinYAccIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlYaccMultiplicator->GetValue();
    spinCtrlYacc->SetIncrement(newIncrement);
}

void MyWindow::OnSpinYAccUpdate(wxSpinEvent& event)
{
    const int yAccCtrlValue = spinCtrlYacc->GetValue();
    yBias = yAccCtrlValue;
}

void MyWindow::OnSpinZAccIncrUpdate(wxSpinEvent& event)
{
    const int newIncrement = spinCtrlZaccMultiplicator->GetValue();
    spinCtrlZacc->SetIncrement(newIncrement);
}

void MyWindow::OnSpinZAccUpdate(wxSpinEvent& event)
{
    const int zAccCtrlValue = spinCtrlZacc->GetValue();
    rawGrawity = zAccCtrlValue;
}

void MyWindow::processFiltration(MeasurementsController& rawMeasurement, const uint32_t deltaTimeMs)
{
    if (kalmanFilterSetupGui.getIsCallibrationDone() == false)
    {
        magnChartGui.updateChart(magnPointsBuffer, filteredAzimuthBuffer, rollBuffer, pitchBuffer, expectedOrientationBuffer, gpsAzimuthBuffer,
            rawMeasurement.getRawXMagn(), rawMeasurement.getRawYMagn(), rawMeasurement.getYawFromMagn()*360/(2*M_PI), 1.0, totalTimeMs);

        TransformedAccel transformedAccel{ 0.0,0.0,0.0 };
        updateAccChart(
            transformedAccel,
            rawMeasurement.getXaccMPerS2(),
            rawMeasurement.getYaccMPerS2(),
            rawMeasurement.getZaccMPerS2(),
            rawMeasurement.getCompensatedAccData(),
            std::nullopt, std::nullopt,
            totalTimeMs, deltaTimeMs);

        angleVelocityChartGui.updateChart(xAngleVelocityBuffer, yAngleVelocityBuffer, zAngleVelocityBuffer, filteredZangleVelocityBuffer,
            rawMeasurement.getXangleVelocityDegreePerS(),
            rawMeasurement.getYangleVelocityDegreePerS(),
            rawMeasurement.getZangleVelocityDegreePerS(), 1.0, totalTimeMs);
    }
    else
    {
        if (measurementCounterAfterCallibration == 0)
        {
            totalTimeMs = 0;
            resetChartsAfterCallibration();

        }
        measurementCounterAfterCallibration++;

        const TransformedAccel transformedAccel = accelTransform.transform(rawMeasurement);
        const XyDistance xyDistance = accelTransform.getXyDistance(deltaTimeMs);

        std::optional<GpsDistanceAngular>gpsDistanceAngular = std::nullopt;
        if (currentGpsMeasurements.first == true) //new GPS data is available
        {
            const double lon{ gpsDataConverter.getLongitude() };
            const double lat{ gpsDataConverter.getLatitude() };
            const double velocity{ gpsDataConverter.getVelocityMperS() };
            const double angle{ gpsDataConverter.getOrientation() };
            std::optional<GpsDistanceAngular> gpsBasedPosition = haversineConverter.calculateCurrentPosition(lon, lat);
            gpsBasedPosition.value().angle = angle;
            gpsBasedPosition.value().velocity = velocity;
            gpsBasedPosition.value().expectedXposition = gpsDataConverter.getExpectedPositionX();
            gpsBasedPosition.value().expectedYposition = gpsDataConverter.getExpectedPositionY();

            expectedX = gpsDataConverter.getExpectedPositionX();
            expectedY = gpsDataConverter.getExpectedPositionY();

            gpsDistanceAngular = gpsBasedPosition;

            const auto distance = sqrt(pow(gpsDistanceAngular.value().xPosition - previousPositionXYfromGps.first, 2) 
                + pow(gpsDistanceAngular.value().yPosition - previousPositionXYfromGps.second, 2));

            gpsDistanceAngular.value().distanceInPeriod = distance;

            currentGpsBasedPosition.first = gpsBasedPosition.value().xPosition;
            currentGpsBasedPosition.second = gpsBasedPosition.value().yPosition;
            currentGpsAzimuth = angle;
            currentGpsMeasurements.first = false;
        }

        kalmanFilters.makeAzimuthFitration(
            gpsDistanceAngular,
            rawMeasurement.getZangleVelocityDegreePerS(),
            rawMeasurement.getAzimuth(),
            deltaTimeMs);

        const double filteredAzimuth = kalmanFilters.getFilterForAzimuth().vecX()[0];
        const auto rollFromAcc = rawMeasurement.getRollFromAcc() * (360.0 / (2.0 *M_PI));
        const auto pitchFromAcc = rawMeasurement.getPitchFromAcc() * (360.0 / (2.0*M_PI));
        const auto yawFromMagn = rawMeasurement.getAzimuth();


        if (gpsDistanceAngular)
        {
            gpsDistanceAngular.value().xPositionBasedOnDistanceAndAzimuth = previousPositionXYfromAzimuth.first +
                gpsDistanceAngular.value().distanceInPeriod * cos(filteredAzimuth * M_PI / 180.0);

            gpsDistanceAngular.value().yPositionBasedOnDistanceAndAzimuth = previousPositionXYfromAzimuth.second +
                gpsDistanceAngular.value().distanceInPeriod * sin(filteredAzimuth * M_PI / 180.0);


        }
        const bool doInrement = kalmanFilters.makePositionFiltration(rawMeasurement, currentXposition, currentYposition, gpsDistanceAngular, transformedAccel, filteredAzimuth, rawMeasurement.getXaccMPerS2(), rawMeasurement.getYaccMPerS2(), deltaTimeMs);

        if (gpsDistanceAngular)
        {
            previousPositionXYfromGps.first = gpsDistanceAngular.value().xPosition;
            previousPositionXYfromGps.second = gpsDistanceAngular.value().yPosition;

            previousPositionXYfromAzimuth.first = gpsDistanceAngular.value().xPositionBasedOnDistanceAndAzimuth;
            previousPositionXYfromAzimuth.second = gpsDistanceAngular.value().yPositionBasedOnDistanceAndAzimuth;
        }

        gyroCallibrator.collectDataForCallibration(rawMeasurement, rollFromAcc, pitchFromAcc, filteredAzimuth);

        positionChartGui.updateChart(rawPositionBuffer, rawMeasurement.getXDistance(), rawMeasurement.getYDistance(), totalTimeMs);

        const double filteredZAngleVel = 0.0;


        expectedOrientationBuffer.AddElement(wxRealPoint(totalTimeMs, rawMeasurement.getExpectedOrientation()));
        gpsAzimuthBuffer.AddElement(wxRealPoint(totalTimeMs, currentGpsAzimuth));
        magnChartGui.updateChart(magnPointsBuffer, filteredAzimuthBuffer, rollBuffer, pitchBuffer, expectedOrientationBuffer, gpsAzimuthBuffer,
            rawMeasurement.getRawXMagn(), rawMeasurement.getRawYMagn(), rawMeasurement.getAzimuth(), filteredAzimuth, totalTimeMs);


        const double currentAzimuthRmse = rmseCalcultator.calculateAzimuthRmse(rawMeasurement.getExpectedOrientation(), filteredAzimuth);

        rmseAzimuthBuffer.AddElement(wxRealPoint(totalTimeMs, currentAzimuthRmse));
        

        const auto calculatedPosition{ positionUpdater.getCurrentPosition() };
        const auto totalDistance{ positionUpdater.getTotalDistance() };

        filteredPositionX = kalmanFilters.getFilterForPosition().vecX()(0);//PosX
        filteredPositionY = kalmanFilters.getFilterForPosition().vecX()(1);//PosY

        

        rollPitchChartGui.updateChart(rawMeasurement,
            rollBasedOnAccBuffer, pitchBasedOnAccBuffer, magnPointsBuffer,
            rollBuffer, pitchBuffer, yawBuffer,
            roll, pitch, rawMeasurement.getAzimuth(), totalDistance, totalTimeMs);

        //const double filteredVelocity
        actualVelocity += rawMeasurement.getActualVelocityMperSec();
        const auto xVelocityFiltered{ kalmanFilters.getFilterForPosition().vecX()(2) };
        const auto yVelocityFiltered{ kalmanFilters.getFilterForPosition().vecX()(3) };
        
        if (gpsDistanceAngular)
        {
            appLogger.logFilteredData(filteredAzimuth, filteredPositionX, filteredPositionY);
            actualVelocityFromFilter += sqrt(xVelocityFiltered * xVelocityFiltered + yVelocityFiltered * yVelocityFiltered);
        }
        else
        {
            actualVelocityFromFilter += sqrt(xVelocityFiltered * xVelocityFiltered + yVelocityFiltered * yVelocityFiltered);
        }
        
        const double distanceInPeriod = ((actualVelocityFromFilter + actualVelocity) / 2.0) * (deltaTimeMs / 1000.0);
        actualDistance += distanceInPeriod;

        calculatedVelocityBuffer.AddElement(wxRealPoint(totalTimeMs, actualVelocity));
        velocityFromFilterBuffer.AddElement(wxRealPoint(totalTimeMs, actualVelocityFromFilter));
        velocityChartGui.updateChart(calculatedVelocityBuffer, velocityFromFilterBuffer, actualVelocity, actualVelocityFromFilter, actualDistance);

        const double xAccGyroCompensation = (rawMeasurement.getXaccMPerS2() * cos(roll)) - (rawMeasurement.getZaccMPerS2() * sin(pitch));
        const double yAccGyroCompensation = (rawMeasurement.getYaccMPerS2() * cos(pitch)) + (rawMeasurement.getZaccMPerS2() * sin(roll));

        angleVelocityChartGui.updateChart(xAngleVelocityBuffer, yAngleVelocityBuffer, zAngleVelocityBuffer, filteredZangleVelocityBuffer,
            rawMeasurement.getXangleVelocityDegreePerS(),
            rawMeasurement.getYangleVelocityDegreePerS(),
            rawMeasurement.getZangleVelocityDegreePerS(), filteredZAngleVel, totalTimeMs);



        positionUpdater.updatePosition(filteredPositionX, filteredPositionY, rawMeasurement.getXDistance(), rawMeasurement.getYDistance(),
            filteredAzimuth);
        const std::pair<double,double> position = positionUpdater.updatePosition(distanceInPeriod, filteredAzimuth);

        updateGpsBasedPositionChart(gpsDistanceAngular);

        updateAccChart(
            transformedAccel,
            rawMeasurement.getXaccMPerS2(),
            rawMeasurement.getYaccMPerS2(),
            rawMeasurement.getZaccMPerS2(),
            rawMeasurement.getCompensatedAccData(),
            0.0, 0.0,
            totalTimeMs, deltaTimeMs);

        const double expectedPositionX = rawMeasurement.getExpectedPositionX();
        const double expectedPositionY = rawMeasurement.getExpectedPositionY();
        expectedPositionBuffer.AddElement(wxRealPoint(expectedPositionX, expectedPositionY));



        const double rmsePositionX = rmseCalcultator.positionXrmse;
        const double rmsePositionY = rmseCalcultator.positionYrmse;

        const double positionDiff = rmseCalcultator.calculateDiffExpectedVsFiltered(expectedGpsPositionBuffer, filteredPositionBuffer);

        const double positionDiffExpectedVsGps = rmseCalcultator.calculateDiffBetweenExpectedAndCurrentPosition(
            ExpectedPosition{ expectedY, expectedX },
            currentGpsBasedPosition);

        const double azimuthDiff = rmseCalcultator.calculateDiffBetweenExpectedAndCurrentAzimuth(rawMeasurement.getExpectedOrientation(), filteredAzimuth);


      
        rmsePositionXBuffer.AddElement(wxRealPoint(totalTimeMs, rmsePositionX));
        rmsePositionYBuffer.AddElement(wxRealPoint(totalTimeMs, rmsePositionY));
        

        positionDifferenceBuffer.AddElement(wxRealPoint(totalTimeMs, positionDiff));
        azimuthDifferenceBuffer.AddElement(wxRealPoint(totalTimeMs, std::fabs(azimuthDiff)));
        positionBetweenGpsAndExpectedDifferenceBuffer.AddElement(wxRealPoint(totalTimeMs, positionDiffExpectedVsGps));

        rmseAzimuthChartGui.updateChart(rmseAzimuthBuffer, rmsePositionXBuffer, rmsePositionYBuffer);
        positionDiffChartGui.updateChart(positionDifferenceBuffer, positionBetweenGpsAndExpectedDifferenceBuffer);
        azimuthDiffChartGui.updateChart(azimuthDifferenceBuffer);

        updateFilteredPositionChart(filteredPositionX, filteredPositionY, calculatedPosition, position, actualDistance, filteredAzimuth, totalTimeMs);

    }
}

void MyWindow::OnFilterReceivedDataProcessingTimer(wxTimerEvent& event)
{
    if (kalmanFilterSetupGui.getIsCallibrationDone())
    {

        MeasurementsController rawMeasurement(appLogger, rawGrawity, xBias, yBias,
            angleVelocityChartGui.getXgyroBias(),
            angleVelocityChartGui.getYgyroBias(),
            angleVelocityChartGui.getZgyroBias(), magnetometerCallibrator);
        const uint32_t deltaTimeMs = deltaTimeCalculator.getDurationInMs();
        totalTimeMs += static_cast<double>(deltaTimeMs);

        if (currentSensorMeasurements.first && rawMeasurement.assign(currentSensorMeasurements.second, expectedPositionAsStrings, deltaTimeMs, REAL_TIME_MEASUREMENT, currentGpsMeasurements.first))
        {
            processFiltration(rawMeasurement, deltaTimeMs);
            currentSensorMeasurements.first = false;
            currentGpsMeasurements.first = false; //wait for new valid GPS data
        }
        else
        {
            const std::string errThreadEvent{ "INF timer expired - no valid measurements data available for current filtration cycle - only estimation based on previous system state" };
            appLogger.logErrThreadDataHandling(errThreadEvent);
        }
    }
}

void MyWindow::OnFilterReceivedGpsProcessingTimer(wxTimerEvent& event)
{
    if (currentGpsMeasurements.first) //new GPS data available
    {
        if (not gpsDataConverter.handleGpsData(currentGpsMeasurements.second, false))
        {
            const std::string errThreadEvent{ "INF timer expired - no valid GPS data available for current filtration cycle - only estimation based on previous system state" };
            appLogger.logErrThreadDataHandling(errThreadEvent);
        }
    }
    else
    {
        const std::string errThreadEvent{ "INF timer expired - no new GPS data available for current filtration cycle - only estimation based on previous system state" };
        appLogger.logErrThreadDataHandling(errThreadEvent);
    }
}

void MyWindow::OnFilterFileMeasTimer(wxTimerEvent& event)
{
    if (kalmanFilterSetupGui.getIsCallibrationDone() == true && kalmanFilterSetupGui.getIsRestartFiltrationNeeded() == true)
    {
        csvMeasurementReader.setReadMeasurementFromBegining();
        csvMeasurementReader.setReadGpsDataFromBegining();
        kalmanFilterSetupGui.setIsRestartFiltrationNeeded(false);
    }
    const std::vector<std::string>& measurements = csvMeasurementReader.readCSVrow();
    
    currentSensorMeasurements.first = true;
    currentSensorMeasurements.second = measurements;
    
    if (measurements.empty())
    {
        return;
    }
    const auto isGpsDataString{ measurements[measurements.size() - 1] };
    if (isGpsDataString == "1")
    {
        const std::vector<std::string>& gpsData = csvMeasurementReader.readCSVrowGpsData();
        currentGpsMeasurements.first = true; //new GPS data ready for processing
        currentGpsMeasurements.second = gpsData;
        if (not gpsDataConverter.handleGpsData(currentGpsMeasurements.second, true))
        {
            const std::string errThreadEvent{ "INF - no valid GPS data available for current filtration cycle - only estimation based on previous system state" };
            appLogger.logErrThreadDataHandling(errThreadEvent);
        }
    }
}

//every time when sensors data received throughout WIFI
void MyWindow::OnSensorsDataThreadEvent(wxThreadEvent& event) 
{
    MeasurementCustomizator* myEvent = dynamic_cast<MeasurementCustomizator*>(&event);
    if (myEvent)
    {
        const std::vector<std::string>& measurements = myEvent->GetStringVector();

        appLogger.logReceivedDataOnMainThread(measurements);
        //DURING CALLIBRATION
        if (not kalmanFilterSetupGui.getIsCallibrationDone())
        {
            if (measurements.size() == 10)
            {
                if (isFirstMeasurement)
                {
                    deltaTimeCalculator.startTimer();
                    isFirstMeasurement = false;
                    return;
                }
                const uint32_t deltaTimeMs = deltaTimeCalculator.getDurationInMs();
                const int16_t xMagnOffset = magnetometerCallibrator.getXoffset();
                const int16_t yMagnOffset = magnetometerCallibrator.getYoffset();
                MeasurementsController rawMeasurement(appLogger, rawGrawity, xBias, yBias,
                    angleVelocityChartGui.getXgyroBias(),
                    angleVelocityChartGui.getYgyroBias(),
                    angleVelocityChartGui.getZgyroBias(), magnetometerCallibrator);
                totalTimeMs += static_cast<double>(deltaTimeMs);
                if (rawMeasurement.assign(measurements, expectedPositionAsStrings, deltaTimeMs, REAL_TIME_MEASUREMENT, false))
                {
                    processFiltration(rawMeasurement, deltaTimeMs);
                }
            }
            else
            {
                const std::string errThreadEvent{ "ERR when handling data from thread/timer during callibration process for WIFI reception - wrong size of data - should be 10!!! Currently the size is "
                    + std::to_string(measurements.size())};
                appLogger.logErrThreadDataHandling(errThreadEvent);
            }
        }
        else //AFTER CALLIBRATION
        {
            //new data available for processing, save data for timer expiration
            currentSensorMeasurements.first = true;
            currentSensorMeasurements.second = measurements;
        }
    }
    else
    {
        const std::string errThreadEvent{ "ERR when handling data from thread - no event received!!!" };
        appLogger.logErrThreadDataHandling(errThreadEvent);
    }
}

void MyWindow::OnGpsDataThreadEvent(wxThreadEvent& event)
{
    GpsDataCustomizator* myEvent = dynamic_cast<GpsDataCustomizator*>(&event);
    if (myEvent)
    {
        const std::vector<std::string>& measurements = myEvent->GetStringVector();

        if (not kalmanFilterSetupGui.getIsCallibrationDone())
        {
            if (not gpsDataConverter.handleGpsData(measurements, false))
            {
                const std::string errThreadEvent{ "ERR during handling GPS data on callibration process!!!" };
                appLogger.logErrThreadDataHandling(errThreadEvent);
            }
        }
        else
        {
            currentGpsMeasurements.first = true; //new GPS data ready for processing
            currentGpsMeasurements.second = measurements;
            appLogger.logGpsCsvData(measurements);
        }
        appLogger.logReceivedDataOnMainThread(measurements, " GPS");
    }
    else
    {
        const std::string errThreadEvent{ "ERR when handling data from thread - no event received!!!" };
        appLogger.logErrThreadDataHandling(errThreadEvent);
    }
}

//every time when sensors data received throughout COM port
void MyWindow::OnSensorDataComThreadEvent(wxThreadEvent& event)
{
    MeasurementCustomizator* myEvent = dynamic_cast<MeasurementCustomizator*>(&event);
    if (myEvent)
    {
        const std::vector<std::string>& measurements = myEvent->GetStringVector();

        appLogger.logReceivedDataOnMainThread(measurements);
        //DURING CALLIBRATION
        if (not kalmanFilterSetupGui.getIsCallibrationDone())
        {
            if (measurements.size() == 9)
            {
                if (isFirstMeasurement)
                {
                    deltaTimeCalculator.startTimer();
                    isFirstMeasurement = false;
                    return;
                }
                const uint32_t deltaTimeMs = deltaTimeCalculator.getDurationInMs();
                const int16_t xMagnOffset = magnetometerCallibrator.getXoffset();
                const int16_t yMagnOffset = magnetometerCallibrator.getYoffset();
                MeasurementsController rawMeasurement(appLogger, rawGrawity, xBias, yBias,
                    angleVelocityChartGui.getXgyroBias(),
                    angleVelocityChartGui.getYgyroBias(),
                    angleVelocityChartGui.getZgyroBias(), magnetometerCallibrator);
                totalTimeMs += static_cast<double>(deltaTimeMs);
                if (rawMeasurement.assign(measurements, expectedPositionAsStrings, deltaTimeMs, false, false))
                {
                    processFiltration(rawMeasurement, deltaTimeMs);
                }
            }
            else
            {
                const std::string errThreadEvent{ "ERR when handling data from thread/timer during callibration process for COM port reception - wrong size of data - should be 9!!! Currently the size is "
                    + std::to_string(measurements.size()) };
                appLogger.logErrThreadDataHandling(errThreadEvent);
            }
        }
        else //AFTER CALLIBRATION
        {
            //new sensor data available, save data for timer expiration
            currentSensorMeasurements.first = true;
            currentSensorMeasurements.second = measurements;
        }
    }
    else
    {
        const std::string errThreadEvent{ "ERR when handling data from thread - no event received!!!" };
        appLogger.logErrThreadDataHandling(errThreadEvent);
    }
}

void MyWindow::resetChartsAfterCallibration()
{
    magnPointsBuffer.Clear();
    filteredAzimuthBuffer.Clear();

    xAccBuffer.Clear();
    yAccBuffer.Clear();
    zAccBuffer.Clear();
    xAccGravityCompensationBuffer.Clear();
    yAccGravityCompensationBuffer.Clear();
    zAccGravityCompensationBuffer.Clear();

    compensatedXAccDataBuffer.Clear();
    compensatedYAccDataBuffer.Clear();
    filteredXaccBuffer.Clear();
    filteredYaccBuffer.Clear();
    xAccWithGyroCompensation.Clear();
    yAccWithGyroCompensation.Clear();

    xAngleVelocityBuffer.Clear();
    yAngleVelocityBuffer.Clear();
    zAngleVelocityBuffer.Clear();
    filteredZangleVelocityBuffer.Clear();

    rollBuffer.Clear();
    pitchBuffer.Clear();
    rollBasedOnAccBuffer.Clear();
    pitchBasedOnAccBuffer.Clear();

    rawPositionBuffer.Clear();
    filteredPositionBuffer.Clear();
    calculatedPositionBuffer.Clear();
    realPositionBuffer.Clear();
    gpsBasedPositionBuffer.Clear();
    gpsBasedPositionWithAzimuthBuffer.Clear();
    calculatedVelocityBuffer.Clear();
    velocityFromFilterBuffer.Clear();
    expectedPositionBuffer.Clear();
    expectedGpsPositionBuffer.Clear();

    rmseAzimuthBuffer.Clear();
    rmsePositionXBuffer.Clear();
    rmsePositionYBuffer.Clear();
    positionDifferenceBuffer.Clear();
    azimuthDifferenceBuffer.Clear();
    gpsAzimuthBuffer.Clear();
}

void MyWindow::updateAccChart(const TransformedAccel& transformedAccel, const double xAccMPerS2, const double yAccMPerS2, const double zAccMPerS2,
    const CompensatedAccData& compensatedAccData,
    const std::optional<double> filteredXacc, const std::optional<double> filteredYacc,
    const double timeMs, const uint32_t deltaTime)
{
    xAccValue->SetLabel(std::to_string(xAccMPerS2));
    yAccValue->SetLabel(std::to_string(yAccMPerS2));
    zAccValue->SetLabel(std::to_string(zAccMPerS2));

    deltaTimeValue->SetLabel(std::to_string(deltaTime));
    totalTimeValue->SetLabel(std::to_string(timeMs));

    xAccBuffer.AddElement(wxRealPoint(timeMs, xAccMPerS2));
    yAccBuffer.AddElement(wxRealPoint(timeMs, yAccMPerS2));
    filteredYaccBuffer.AddElement(wxRealPoint(timeMs, filteredYacc.value_or(-3)));

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(xAccBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(yAccBuffer.getBuffer()));

    dataset->AddSerie(new XYSerie(zAccBuffer.getBuffer()));

    dataset->GetSerie(0)->SetName("Raw X acceleration");
    dataset->GetSerie(1)->SetName("Raw Y acceleration");
    dataset->GetSerie(2)->SetName("Raw Z acceleration");

    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Acceleration [m/s2]"));
    bottomAxis->SetTitle(wxT("time [ms]"));
    if (xAccBuffer.getBuffer().size() >= 100)
    {
        bottomAxis->SetFixedBounds(xAccBuffer.getBuffer()[0].x, xAccBuffer.getBuffer()[99].x);
    }

    Legend* legend = new Legend(wxTOP, wxRIGHT);
    plot->SetLegend(legend);
    plot->AddObjects(dataset, leftAxis, bottomAxis);

    Chart* chart = new Chart(plot, "Acceleration");

    accChartPanel->SetChart(chart);
}

void MyWindow::updateGpsBasedPositionChart(const std::optional<GpsDistanceAngular> gpsBasedPosition)
{
    if (gpsBasedPosition)
    {
        XYPlot* plot = new XYPlot();
        XYSimpleDataset* dataset = new XYSimpleDataset();
        gpsBasedPositionBuffer.AddElement(wxRealPoint(gpsBasedPosition.value().yPosition, gpsBasedPosition.value().xPosition));
        expectedGpsPositionBuffer.AddElement(wxRealPoint(gpsBasedPosition.value().expectedXposition, gpsBasedPosition.value().expectedYposition));
        dataset->AddSerie(new XYSerie(gpsBasedPositionBuffer.getBuffer()));
        dataset->AddSerie(new XYSerie(expectedGpsPositionBuffer.getBuffer()));
        dataset->AddSerie(new XYSerie(filteredPositionBuffer.getBuffer()));
        dataset->GetSerie(0)->SetName("Gps position");
        dataset->GetSerie(1)->SetName("Expected position");
        dataset->GetSerie(2)->SetName("Filtered position");
        dataset->SetRenderer(new XYLineRenderer());
        NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
        NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
        leftAxis->SetTitle(wxT("Y [m]"));
        bottomAxis->SetTitle(wxT("X [m]"));
        if (MovementModel::PEDESTRIAN == kalmanFilterSetupGui.getMovementModel())
        {
            leftAxis->SetFixedBounds(-2, 10);
            bottomAxis->SetFixedBounds(-2, 10);
        }
        else if (MovementModel::CAR == kalmanFilterSetupGui.getMovementModel())
        {
            leftAxis->SetFixedBounds(-140, 10);
            bottomAxis->SetFixedBounds(-100, 80);
        }

        plot->AddObjects(dataset, leftAxis, bottomAxis);
        Legend* legend = new Legend(wxTOP, wxRIGHT);
        plot->SetLegend(legend);
        Chart* chart = new Chart(plot, "Position - estimated vs expected");
        gpsBasedPositionChartPanel->SetChart(chart);
    }
    else
    {
        XYPlot* plot = new XYPlot();
        XYSimpleDataset* dataset = new XYSimpleDataset();
        dataset->AddSerie(new XYSerie(gpsBasedPositionBuffer.getBuffer()));
        dataset->AddSerie(new XYSerie(expectedGpsPositionBuffer.getBuffer()));
        dataset->AddSerie(new XYSerie(filteredPositionBuffer.getBuffer()));
        dataset->GetSerie(0)->SetName("Gps position");
        dataset->GetSerie(1)->SetName("Expected position");
        dataset->GetSerie(2)->SetName("Filtered position");
        dataset->SetRenderer(new XYLineRenderer());
        NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
        NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
        leftAxis->SetTitle(wxT("Y [m]"));
        bottomAxis->SetTitle(wxT("X [m]"));
        if (MovementModel::PEDESTRIAN == kalmanFilterSetupGui.getMovementModel())
        {
            leftAxis->SetFixedBounds(-2, 10);
            bottomAxis->SetFixedBounds(-2, 10);
        }
        else if (MovementModel::CAR == kalmanFilterSetupGui.getMovementModel())
        {
            leftAxis->SetFixedBounds(-140, 10);
            bottomAxis->SetFixedBounds(-100, 80);
        }
        plot->AddObjects(dataset, leftAxis, bottomAxis);
        Legend* legend = new Legend(wxTOP, wxRIGHT);
        plot->SetLegend(legend);
        Chart* chart = new Chart(plot, "Position - estimated vs expected");
        gpsBasedPositionChartPanel->SetChart(chart);
    }
}

void MyWindow::updateFilteredPositionChart(const double filteredPositionX, const double filteredPositionY,
    const std::pair<double, double> calculatedPosition,
    const std::pair<double, double> position,
    const double actualDistance, const double filteredAzimuth, const double timeMs)
{
    currentFilteredXPosition = filteredPositionX;
    currentFilteredYPosition = filteredPositionY;
    currentXposition = position.first;
    currentYposition = position.second;

    filteredPositionBuffer.AddElement(wxRealPoint(currentFilteredYPosition, currentFilteredXPosition));
    calculatedPositionBuffer.AddElement(wxRealPoint(calculatedPosition.first, calculatedPosition.second));
    realPositionBuffer.AddElement(wxRealPoint(currentXposition, currentYposition));


    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(filteredPositionBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(rawPositionBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(calculatedPositionBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(realPositionBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(expectedPositionBuffer.getBuffer()));

    dataset->GetSerie(0)->SetName("filtered position");
    dataset->GetSerie(1)->SetName("raw position");
    dataset->GetSerie(2)->SetName("calculated position");
    dataset->GetSerie(3)->SetName("real position");
    dataset->GetSerie(4)->SetName("expected position");

    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("Filtered Y position [m]"));
    bottomAxis->SetTitle(wxT("Filtered X position [m]"));

    leftAxis->SetFixedBounds(-500, 250);
    bottomAxis->SetFixedBounds(-500, 250);
    
    DatasetArray datasetArray();
    //datasetArray
    Legend* legend = new Legend(wxTOP, wxRIGHT);
    plot->SetLegend(legend);
    plot->AddObjects(dataset, leftAxis, bottomAxis);
    Chart* chart = new Chart(plot, "Filtered position");

    filteredPositionChartPanel->SetChart(chart);
}

void MyWindow::updateFilteredAngleXVelocityChart(const double filteredXangle, const double measuredXangle, const double time)
{
    currentXangleFiltered += filteredXangle;
    currentXangleMeasured += measuredXangle;

    filteredXAngleVelocityBuffer.AddElement(wxRealPoint(time, filteredXangle));

    XYPlot* plot = new XYPlot();
    XYSimpleDataset* dataset = new XYSimpleDataset();
    dataset->AddSerie(new XYSerie(xAngleVelocityBuffer.getBuffer()));
    dataset->AddSerie(new XYSerie(filteredXAngleVelocityBuffer.getBuffer()));

    dataset->SetRenderer(new XYLineRenderer());
    NumberAxis* leftAxis = new NumberAxis(AXIS_LEFT);
    NumberAxis* bottomAxis = new NumberAxis(AXIS_BOTTOM);
    leftAxis->SetTitle(wxT("X angle velocity [degree/s]"));
    bottomAxis->SetTitle(wxT("Time [ms]"));
    if (filteredXAngleVelocityBuffer.getBuffer().size() >= 100)
    {
        bottomAxis->SetFixedBounds(filteredXAngleVelocityBuffer.getBuffer()[0].x, filteredXAngleVelocityBuffer.getBuffer()[99].x);
    }
    DatasetArray datasetArray();
    Legend* lengend = new Legend(10, 10);
    wxRect rect(wxSize(10, 10));

    plot->AddObjects(dataset, leftAxis, bottomAxis);
    Chart* chart = new Chart(plot, "Filtered/measured angle velocity");

    filteredAngleXVelocity->SetChart(chart);
}

void MyWindow::createSensorDataReceptionThread()
{
    sensorDataReceptionThread = new SensorDataReceptionThread(appLogger, this);
    if (sensorDataReceptionThread->Create() != wxTHREAD_NO_ERROR)
    {
        const std::string threadNotCreated{ "Can't create SensorDataReceptionThread thread! \n" };
        appLogger.logSerialCommStartThread(threadNotCreated);
    }
    else {
        if (sensorDataReceptionThread->Run() != wxTHREAD_NO_ERROR)
        {
            const std::string cantStartThread{ "Can't start SensorDataReceptionThread thread! \n" };
            appLogger.logSerialCommStartThread(cantStartThread);
        }
        else
        {
            const std::string threadStarted{ "New thread SensorDataReceptionThread started.\n" };
            appLogger.logSerialCommStartThread(threadStarted);
            Bind(wxEVT_MY_THREAD_EVENT, &MyWindow::OnSensorsDataThreadEvent, this);
        }
    }
}

void MyWindow::createSensorDataCOMReceptionThread()
{
    sensorDataComReceptionThread = new SensorDataComReceptionThread(appLogger, this);
    if (sensorDataComReceptionThread->Create() != wxTHREAD_NO_ERROR)
    {
        const std::string threadNotCreated{ "Can't create SensorDataComReceptionThread thread! \n" };
        appLogger.logSerialCommStartThread(threadNotCreated);
    }
    else
    {
        if (sensorDataComReceptionThread->Run() != wxTHREAD_NO_ERROR)
        {
            const std::string cantStartThread{ "Can't start SensorDataComReceptionThread thread! \n" };
            appLogger.logSerialCommStartThread(cantStartThread);
        }
        else
        {
            const std::string threadStarted{ "New thread SensorDataComReceptionThread started.\n" };
            appLogger.logSerialCommStartThread(threadStarted);
            Bind(wxEVT_MY_THREAD_EVENT_2, &MyWindow::OnSensorDataComThreadEvent, this);
        }
    }
}

void MyWindow::prepareAccChart()
{
    wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
    accPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
    wxPanel* controlPanel = new wxPanel(accPanelSplitter, wxID_ANY);

    accChartPanel = new wxChartPanel(accPanelSplitter);

    sizerAccPlot = new wxBoxSizer(wxVERTICAL);
    accChartPanel->SetMinSize(wxSize(600, 600));

    wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* controlPanelSizerForXAdj = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* controlPanelSizerForYAdj = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* controlPanelSizerForZAdj = new wxBoxSizer(wxHORIZONTAL);

    spinCtrlXacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -17000, 17000, 15700);
    spinCtrlXacc->SetIncrement(100);
    spinCtrlXaccMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 100);
    wxStaticText* xAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust X acc");
    spinCtrlXacc->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinXAccUpdate, this);
    spinCtrlXaccMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinXAccIncrUpdate, this);

    spinCtrlYacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -17000, 17000, 675);
    spinCtrlYacc->SetIncrement(100);
    spinCtrlYaccMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 100);
    wxStaticText* yAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Y acc");
    spinCtrlYacc->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinYAccUpdate, this);
    spinCtrlYaccMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinYAccIncrUpdate, this);

    spinCtrlZacc = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -17000, 17000, 16100);
    spinCtrlZacc->SetIncrement(100);
    spinCtrlZaccMultiplicator = new wxSpinCtrl(controlPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -1000, 1000, 100);
    wxStaticText* zAccText = new wxStaticText(controlPanel, wxID_ANY, "Adjust Z acc");
    spinCtrlZacc->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinZAccUpdate, this);
    spinCtrlZaccMultiplicator->Bind(wxEVT_SPINCTRL, &MyWindow::OnSpinZAccIncrUpdate, this);

    controlPanelSizer->Add(xAccText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForXAdj->Add(spinCtrlXacc, 0, wxALL| wxALIGN_CENTER, 5);
    controlPanelSizerForXAdj->Add(spinCtrlXaccMultiplicator, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(controlPanelSizerForXAdj, 0, wxALL | wxALIGN_CENTER, 5);

    controlPanelSizer->Add(yAccText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForYAdj->Add(spinCtrlYacc, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForYAdj->Add(spinCtrlYaccMultiplicator, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(controlPanelSizerForYAdj, 0, wxALL | wxALIGN_CENTER, 5);

    controlPanelSizer->Add(zAccText, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForZAdj->Add(spinCtrlZacc, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizerForZAdj->Add(spinCtrlZaccMultiplicator, 0, wxALL | wxALIGN_CENTER, 5);
    controlPanelSizer->Add(controlPanelSizerForZAdj, 0, wxALL | wxALIGN_CENTER, 5);


    wxBoxSizer* accSetupButtonsSizer = new wxBoxSizer(wxHORIZONTAL);

    wxButton* resetButton = new wxButton(controlPanel, wxID_ANY, "Reset chart");
    accSetupButtonsSizer->Add(resetButton, 0, wxALL|wxALIGN_LEFT, 5);
    resetButton->Bind(wxEVT_BUTTON, &MyWindow::OnResetAccChart, this);

    wxButton* submitButton = new wxButton(controlPanel, wxID_ANY, "Submit adjustments");
    accSetupButtonsSizer->Add(submitButton, 0, wxALL | wxALIGN_LEFT, 5);
    submitButton->Bind(wxEVT_BUTTON, &MyWindow::OnSubmitAccAdjustments, this);


    wxBoxSizer* xAccLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* yAccLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* zAccLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* deltaTimeLabelsSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* totalTimeLabelsSizer = new wxBoxSizer(wxHORIZONTAL);

    wxStaticText* xAccName = new wxStaticText(controlPanel, wxID_ANY, "Current X acc[ms/s^2]: ");
    wxStaticText* yAccName = new wxStaticText(controlPanel, wxID_ANY, "Current Y acc[ms/s^2]: ");
    wxStaticText* zAccName = new wxStaticText(controlPanel, wxID_ANY, "Current Z acc[ms/s^2]: ");
    xAccValue = new wxStaticText(controlPanel, wxID_ANY, "0");
    yAccValue = new wxStaticText(controlPanel, wxID_ANY, "0");
    zAccValue = new wxStaticText(controlPanel, wxID_ANY, "0");

    wxStaticText* deltaTimeName = new wxStaticText(controlPanel, wxID_ANY, "Delta time[ms]: ");
    wxStaticText* totalTimeName = new wxStaticText(controlPanel, wxID_ANY, "Total time[ms]: ");
    deltaTimeValue = new wxStaticText(controlPanel, wxID_ANY, "0");
    totalTimeValue = new wxStaticText(controlPanel, wxID_ANY, "0");

    xAccLabelsSizer->Add(xAccName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    xAccLabelsSizer->Add(xAccValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(xAccLabelsSizer, 0, wxALL, 5);

    yAccLabelsSizer->Add(yAccName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    yAccLabelsSizer->Add(yAccValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(yAccLabelsSizer, 0, wxALL, 5);

    zAccLabelsSizer->Add(zAccName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    zAccLabelsSizer->Add(zAccValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(zAccLabelsSizer, 0, wxALL, 5);

    deltaTimeLabelsSizer->Add(deltaTimeName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    deltaTimeLabelsSizer->Add(deltaTimeValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(deltaTimeLabelsSizer, 0, wxALL, 5);

    totalTimeLabelsSizer->Add(totalTimeName, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    totalTimeLabelsSizer->Add(totalTimeValue, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);
    controlPanelSizer->Add(totalTimeLabelsSizer, 0, wxALL, 5);

    controlPanelSizer->Add(accSetupButtonsSizer, 0, wxALL, 5);

    controlPanel->SetSizer(controlPanelSizer);


    accPanelSplitter->SplitVertically(accChartPanel, controlPanel);
    sizerAccPlot->Add(accPanelSplitter, 1, wxEXPAND | wxALL, 5);


    //CreateStatusBar();
    //SetStatusText("wxWidgets Splitter Example");
    panel->SetSizer(sizerAccPlot);
   // m_notebook->AddPage(panel, "Filtered position");

    m_notebook->AddPage(panel, "Acc chart");
}

void MyWindow::prepareGpsBasedPositionChart()
{
    wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
    gpsBasedPositionPanelSplitter = new wxSplitterWindow(panel, wxID_ANY);
    wxPanel* controlPanel = new wxPanel(gpsBasedPositionPanelSplitter, wxID_ANY);

    gpsBasedPositionChartPanel = new wxChartPanel(gpsBasedPositionPanelSplitter);

    wxBoxSizer* sizerGpsBasedPositionPlot = new wxBoxSizer(wxVERTICAL);
    gpsBasedPositionChartPanel->SetMinSize(wxSize(600, 600));

    wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);

    gpsBasedPositionPanelSplitter->SplitVertically(gpsBasedPositionChartPanel, controlPanel);
    sizerGpsBasedPositionPlot->Add(gpsBasedPositionPanelSplitter, 1, wxEXPAND | wxALL, 5);

    panel->SetSizer(sizerGpsBasedPositionPlot);

    m_notebook->AddPage(panel, "GPS position");

}



void MyWindow::prepareVelChart()
{
    //velChartPanel = new wxChartPanel(m_notebook);
    //m_notebook->AddPage(velChartPanel, "Vel chart");
}

void MyWindow::prepareFilteredVelocityChart()
{
    //filteredVelocityChartPanel = new wxChartPanel(m_notebook);
    //m_notebook->AddPage(filteredVelocityChartPanel, "Filtered velocity");
}


void MyWindow::prepareFilteredAngleXVelocityChart()
{
    filteredAngleXVelocity = new wxChartPanel(m_notebook);
    m_notebook->AddPage(filteredAngleXVelocity, "Filtered X angle velocity");
}

void MyWindow::prepareFilteredPositionChart()
{
    wxPanel* panel = new wxPanel(m_notebook, wxID_ANY);
    splitter = new wxSplitterWindow(panel, wxID_ANY);

    wxPanel* controlPanel = new wxPanel(splitter, wxID_ANY);

    filteredPositionChartPanel = new wxChartPanel(splitter);
    sizerPositionPlot = new wxBoxSizer(wxVERTICAL);
    filteredPositionChartPanel->SetMinSize(wxSize(1000, 600));
    wxBoxSizer* controlPanelSizer = new wxBoxSizer(wxVERTICAL);

    controlPanel->SetSizer(controlPanelSizer);


    splitter->SplitVertically(filteredPositionChartPanel, controlPanel);
    sizerPositionPlot->Add(splitter, 1, wxEXPAND | wxALL, 5);

    panel->SetSizer(sizerPositionPlot);
    m_notebook->AddPage(panel, "Filtered position");
}

void MyWindow::prepareGui()
{
    m_notebook = new wxNotebook(this, 1);
    //comSetupPanel = new wxPanel(m_notebook);
    //m_notebook->AddPage(comSetupPanel, "Serial port setup");
    //dataReceptionPanel = new wxPanel(m_notebook);
    //m_notebook->AddPage(dataReceptionPanel, "Data reception");
    
    //innerNotebook = new wxNotebook(kalmanParamsSetupPanel, 2);
    //wxPanel* innerPanel = new wxPanel(innerNotebook);
    //innerNotebook->AddPage(innerPanel, "Inner");

    
    kalmanFilterSetupGui.setup(m_notebook);

    csvMeasurementLoadPanel = new wxPanel(m_notebook);
    m_notebook->AddPage(csvMeasurementLoadPanel, "Load CSV");
    csvMeasurementLoadGui.setup(csvMeasurementLoadPanel);

    prepareAccChart();
    prepareVelChart();
    positionChartGui.setup(m_notebook);
    prepareGpsBasedPositionChart();
    angleVelocityChartGui.setup(m_notebook);
    rollPitchChartGui.setup(m_notebook);
    velocityChartGui.setup(m_notebook);
    prepareFilteredPositionChart();
    prepareFilteredVelocityChart();
    prepareFilteredAngleXVelocityChart();
    magnChartGui.setup(m_notebook);
    rmseAzimuthChartGui.setup(m_notebook);
    positionDiffChartGui.setup(m_notebook);
    azimuthDiffChartGui.setup(m_notebook);


    wxSize size(100, 20);

    wxBoxSizer* panelSizer = new wxBoxSizer(wxVERTICAL);


    filterFileMeasTimer.Bind(wxEVT_TIMER, &MyWindow::OnFilterFileMeasTimer, this);
    filterReceivedDataProcessingTimer.Bind(wxEVT_TIMER, &MyWindow::OnFilterReceivedDataProcessingTimer, this);
    filterReceivedGpsProcessingTimer.Bind(wxEVT_TIMER, &MyWindow::OnFilterReceivedGpsProcessingTimer, this);
}
