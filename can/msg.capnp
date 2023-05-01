@0x8e2af1e708af8b8d;

struct CanData {
  address @0 :UInt32;
  busTime @1 :UInt16;
  dat     @2 :Data;
  src     @3 :UInt8;
}


struct Event {
  logMonoTime @0 :UInt64;  # nanoseconds
  valid @67 :Bool = true;

  union {
    # *********** log metadata ***********
    initData @1 :InitData;
    sentinel @73 :Sentinel;

    # *********** bootlog ***********
    boot @60 :Boot;

    # ********** openpilot daemon msgs **********
    gpsNMEA @3 :GPSNMEAData;
    can @5 :List(CanData);
    controlsState @7 :ControlsState;
    gyroscope @99 :SensorEventData;
    gyroscope2 @100 :SensorEventData;
    accelerometer @98 :SensorEventData;
    accelerometer2 @101 :SensorEventData;
    magnetometer @95 :SensorEventData;
    lightSensor @96 :SensorEventData;
    temperatureSensor @97 :SensorEventData;
    pandaStates @81 :List(PandaState);
    peripheralState @80 :PeripheralState;
    radarState @13 :RadarState;
    liveTracks @16 :List(LiveTracks);
    sendcan @17 :List(CanData);
    liveCalibration @19 :LiveCalibrationData;
    carState @22 :Car.CarState;
    carControl @23 :Car.CarControl;
    longitudinalPlan @24 :LongitudinalPlan;
    lateralPlan @64 :LateralPlan;
    uiPlan @106 :UiPlan;
    ubloxGnss @34 :UbloxGnss;
    ubloxRaw @39 :Data;
    qcomGnss @31 :QcomGnss;
    gpsLocationExternal @48 :GpsLocationData;
    gpsLocation @21 :GpsLocationData;
    gnssMeasurements @91 :GnssMeasurements;
    liveParameters @61 :LiveParametersData;
    liveTorqueParameters @94 :LiveTorqueParametersData;
    cameraOdometry @63 :CameraOdometry;
    thumbnail @66: Thumbnail;
    carEvents @68: List(Car.CarEvent);
    carParams @69: Car.CarParams;
    driverMonitoringState @71: DriverMonitoringState;
    liveLocationKalman @72 :LiveLocationKalman;
    modelV2 @75 :ModelDataV2;
    driverStateV2 @92 :DriverStateV2;
    navModel @104 :NavModelData;

    # camera stuff, each camera state has a matching encode idx
    roadCameraState @2 :FrameData;
    driverCameraState @70: FrameData;
    wideRoadCameraState @74: FrameData;
    roadEncodeIdx @15 :EncodeIndex;
    driverEncodeIdx @76 :EncodeIndex;
    wideRoadEncodeIdx @77 :EncodeIndex;
    qRoadEncodeIdx @90 :EncodeIndex;

    # microphone data
    microphone @103 :Microphone;

    # systems stuff
    androidLog @20 :AndroidLogEntry;
    managerState @78 :ManagerState;
    uploaderState @79 :UploaderState;
    procLog @33 :ProcLog;
    clocks @35 :Clocks;
    deviceState @6 :DeviceState;
    logMessage @18 :Text;
    errorLogMessage @85 :Text;

    # navigation
    navInstruction @82 :NavInstruction;
    navRoute @83 :NavRoute;
    navThumbnail @84: Thumbnail;
    mapRenderState @105: MapRenderState;

    # UI services
    userFlag @93 :UserFlag;
    uiDebug @102 :UIDebug;

    # *********** debug ***********
    testJoystick @52 :Joystick;
    roadEncodeData @86 :EncodeData;
    driverEncodeData @87 :EncodeData;
    wideRoadEncodeData @88 :EncodeData;
    qRoadEncodeData @89 :EncodeData;

    # *********** legacy + deprecated ***********
    model @9 :Legacy.ModelData; # TODO: rename modelV2 and mark this as deprecated
    liveMpcDEPRECATED @36 :LiveMpcData;
    liveLongitudinalMpcDEPRECATED @37 :LiveLongitudinalMpcData;
    liveLocationKalmanDEPRECATED @51 :Legacy.LiveLocationData;
    orbslamCorrectionDEPRECATED @45 :Legacy.OrbslamCorrection;
    liveUIDEPRECATED @14 :Legacy.LiveUI;
    sensorEventDEPRECATED @4 :SensorEventData;
    liveEventDEPRECATED @8 :List(Legacy.LiveEventData);
    liveLocationDEPRECATED @25 :Legacy.LiveLocationData;
    ethernetDataDEPRECATED @26 :List(Legacy.EthernetPacket);
    cellInfoDEPRECATED @28 :List(Legacy.CellInfo);
    wifiScanDEPRECATED @29 :List(Legacy.WifiScan);
    uiNavigationEventDEPRECATED @50 :Legacy.UiNavigationEvent;
    liveMapDataDEPRECATED @62 :LiveMapDataDEPRECATED;
    gpsPlannerPointsDEPRECATED @40 :Legacy.GPSPlannerPoints;
    gpsPlannerPlanDEPRECATED @41 :Legacy.GPSPlannerPlan;
    applanixRawDEPRECATED @42 :Data;
    androidGnssDEPRECATED @30 :Legacy.AndroidGnss;
    lidarPtsDEPRECATED @32 :Legacy.LidarPts;
    navStatusDEPRECATED @38 :Legacy.NavStatus;
    trafficEventsDEPRECATED @43 :List(Legacy.TrafficEvent);
    liveLocationTimingDEPRECATED @44 :Legacy.LiveLocationData;
    liveLocationCorrectedDEPRECATED @46 :Legacy.LiveLocationData;
    navUpdateDEPRECATED @27 :Legacy.NavUpdate;
    orbObservationDEPRECATED @47 :List(Legacy.OrbObservation);
    locationDEPRECATED @49 :Legacy.LiveLocationData;
    orbOdometryDEPRECATED @53 :Legacy.OrbOdometry;
    orbFeaturesDEPRECATED @54 :Legacy.OrbFeatures;
    applanixLocationDEPRECATED @55 :Legacy.LiveLocationData;
    orbKeyFrameDEPRECATED @56 :Legacy.OrbKeyFrame;
    orbFeaturesSummaryDEPRECATED @58 :Legacy.OrbFeaturesSummary;
    featuresDEPRECATED @10 :Legacy.CalibrationFeatures;
    kalmanOdometryDEPRECATED @65 :Legacy.KalmanOdometry;
    uiLayoutStateDEPRECATED @57 :Legacy.UiLayoutState;
    pandaStateDEPRECATED @12 :PandaState;
    driverStateDEPRECATED @59 :DriverStateDEPRECATED;
    sensorEventsDEPRECATED @11 :List(SensorEventData);
  }
}
