
"use strict";

let DebugValue = require('./DebugValue.js');
let CellularStatus = require('./CellularStatus.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let TerrainReport = require('./TerrainReport.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let ActuatorControl = require('./ActuatorControl.js');
let Thrust = require('./Thrust.js');
let GPSRTK = require('./GPSRTK.js');
let GPSRAW = require('./GPSRAW.js');
let Mavlink = require('./Mavlink.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let WaypointList = require('./WaypointList.js');
let CommandCode = require('./CommandCode.js');
let HomePosition = require('./HomePosition.js');
let Waypoint = require('./Waypoint.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let ExtendedState = require('./ExtendedState.js');
let VFR_HUD = require('./VFR_HUD.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let RCIn = require('./RCIn.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let Param = require('./Param.js');
let Trajectory = require('./Trajectory.js');
let RTKBaseline = require('./RTKBaseline.js');
let RCOut = require('./RCOut.js');
let HilSensor = require('./HilSensor.js');
let Altitude = require('./Altitude.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let PositionTarget = require('./PositionTarget.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let MountControl = require('./MountControl.js');
let GPSINPUT = require('./GPSINPUT.js');
let WaypointReached = require('./WaypointReached.js');
let RadioStatus = require('./RadioStatus.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let StatusText = require('./StatusText.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let VehicleInfo = require('./VehicleInfo.js');
let ESCStatus = require('./ESCStatus.js');
let HilGPS = require('./HilGPS.js');
let Tunnel = require('./Tunnel.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let ParamValue = require('./ParamValue.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let SysStatus = require('./SysStatus.js');
let HilControls = require('./HilControls.js');
let ESCInfo = require('./ESCInfo.js');
let RTCM = require('./RTCM.js');
let FileEntry = require('./FileEntry.js');
let State = require('./State.js');
let Vibration = require('./Vibration.js');
let ManualControl = require('./ManualControl.js');
let LandingTarget = require('./LandingTarget.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let BatteryStatus = require('./BatteryStatus.js');
let LogData = require('./LogData.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let LogEntry = require('./LogEntry.js');

module.exports = {
  DebugValue: DebugValue,
  CellularStatus: CellularStatus,
  WheelOdomStamped: WheelOdomStamped,
  MagnetometerReporter: MagnetometerReporter,
  ESCTelemetry: ESCTelemetry,
  TerrainReport: TerrainReport,
  OpticalFlowRad: OpticalFlowRad,
  ActuatorControl: ActuatorControl,
  Thrust: Thrust,
  GPSRTK: GPSRTK,
  GPSRAW: GPSRAW,
  Mavlink: Mavlink,
  ESCInfoItem: ESCInfoItem,
  WaypointList: WaypointList,
  CommandCode: CommandCode,
  HomePosition: HomePosition,
  Waypoint: Waypoint,
  CameraImageCaptured: CameraImageCaptured,
  ExtendedState: ExtendedState,
  VFR_HUD: VFR_HUD,
  ESCStatusItem: ESCStatusItem,
  RCIn: RCIn,
  PlayTuneV2: PlayTuneV2,
  CompanionProcessStatus: CompanionProcessStatus,
  Param: Param,
  Trajectory: Trajectory,
  RTKBaseline: RTKBaseline,
  RCOut: RCOut,
  HilSensor: HilSensor,
  Altitude: Altitude,
  HilActuatorControls: HilActuatorControls,
  PositionTarget: PositionTarget,
  ESCTelemetryItem: ESCTelemetryItem,
  MountControl: MountControl,
  GPSINPUT: GPSINPUT,
  WaypointReached: WaypointReached,
  RadioStatus: RadioStatus,
  CamIMUStamp: CamIMUStamp,
  StatusText: StatusText,
  ADSBVehicle: ADSBVehicle,
  VehicleInfo: VehicleInfo,
  ESCStatus: ESCStatus,
  HilGPS: HilGPS,
  Tunnel: Tunnel,
  OverrideRCIn: OverrideRCIn,
  AttitudeTarget: AttitudeTarget,
  ParamValue: ParamValue,
  NavControllerOutput: NavControllerOutput,
  SysStatus: SysStatus,
  HilControls: HilControls,
  ESCInfo: ESCInfo,
  RTCM: RTCM,
  FileEntry: FileEntry,
  State: State,
  Vibration: Vibration,
  ManualControl: ManualControl,
  LandingTarget: LandingTarget,
  HilStateQuaternion: HilStateQuaternion,
  BatteryStatus: BatteryStatus,
  LogData: LogData,
  EstimatorStatus: EstimatorStatus,
  TimesyncStatus: TimesyncStatus,
  GlobalPositionTarget: GlobalPositionTarget,
  OnboardComputerStatus: OnboardComputerStatus,
  LogEntry: LogEntry,
};
