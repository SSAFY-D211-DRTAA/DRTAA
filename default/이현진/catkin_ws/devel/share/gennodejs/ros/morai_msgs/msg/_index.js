
"use strict";

let DillyCmdResponse = require('./DillyCmdResponse.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let SVADC = require('./SVADC.js');
let Conveyor = require('./Conveyor.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let ExternalForce = require('./ExternalForce.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let PRStatus = require('./PRStatus.js');
let WheelControl = require('./WheelControl.js');
let VehicleCollision = require('./VehicleCollision.js');
let SensorPosControl = require('./SensorPosControl.js');
let Obstacle = require('./Obstacle.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let TrafficLight = require('./TrafficLight.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let IntersectionControl = require('./IntersectionControl.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let ManipulatorControl = require('./ManipulatorControl.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let MapSpec = require('./MapSpec.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let Transforms = require('./Transforms.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let GVStateCmd = require('./GVStateCmd.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let TOF = require('./TOF.js');
let CtrlCmd = require('./CtrlCmd.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let CollisionData = require('./CollisionData.js');
let ERP42Info = require('./ERP42Info.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let GPSMessage = require('./GPSMessage.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let RadarDetection = require('./RadarDetection.js');
let ReplayInfo = require('./ReplayInfo.js');
let RobotOutput = require('./RobotOutput.js');
let Obstacles = require('./Obstacles.js');
let WaitForTick = require('./WaitForTick.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let CMDConveyor = require('./CMDConveyor.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let ShipState = require('./ShipState.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let EventInfo = require('./EventInfo.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let RadarDetections = require('./RadarDetections.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let GhostMessage = require('./GhostMessage.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let PREvent = require('./PREvent.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let RobotState = require('./RobotState.js');
let ObjectStatus = require('./ObjectStatus.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let VelocityCmd = require('./VelocityCmd.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let DillyCmd = require('./DillyCmd.js');
let IntscnTL = require('./IntscnTL.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let VehicleSpec = require('./VehicleSpec.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let Lamps = require('./Lamps.js');
let SaveSensorData = require('./SaveSensorData.js');

module.exports = {
  DillyCmdResponse: DillyCmdResponse,
  SyncModeAddObject: SyncModeAddObject,
  SVADC: SVADC,
  Conveyor: Conveyor,
  MoraiSimProcStatus: MoraiSimProcStatus,
  GeoVector3Message: GeoVector3Message,
  ExternalForce: ExternalForce,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  VehicleSpecIndex: VehicleSpecIndex,
  PRStatus: PRStatus,
  WheelControl: WheelControl,
  VehicleCollision: VehicleCollision,
  SensorPosControl: SensorPosControl,
  Obstacle: Obstacle,
  PRCtrlCmd: PRCtrlCmd,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  TrafficLight: TrafficLight,
  ShipCtrlCmd: ShipCtrlCmd,
  SyncModeRemoveObject: SyncModeRemoveObject,
  MoraiSrvResponse: MoraiSrvResponse,
  IntersectionControl: IntersectionControl,
  ObjectStatusListExtended: ObjectStatusListExtended,
  FaultInjection_Tire: FaultInjection_Tire,
  ObjectStatusList: ObjectStatusList,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  ManipulatorControl: ManipulatorControl,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  MapSpec: MapSpec,
  WaitForTickResponse: WaitForTickResponse,
  EgoVehicleStatus: EgoVehicleStatus,
  SyncModeSetGear: SyncModeSetGear,
  MapSpecIndex: MapSpecIndex,
  Transforms: Transforms,
  SyncModeResultResponse: SyncModeResultResponse,
  GVStateCmd: GVStateCmd,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  MoraiSimProcHandle: MoraiSimProcHandle,
  TOF: TOF,
  CtrlCmd: CtrlCmd,
  NpcGhostInfo: NpcGhostInfo,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  FaultInjection_Controller: FaultInjection_Controller,
  MoraiTLInfo: MoraiTLInfo,
  CollisionData: CollisionData,
  ERP42Info: ERP42Info,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  FaultInjection_Sensor: FaultInjection_Sensor,
  GPSMessage: GPSMessage,
  NpcGhostCmd: NpcGhostCmd,
  RadarDetection: RadarDetection,
  ReplayInfo: ReplayInfo,
  RobotOutput: RobotOutput,
  Obstacles: Obstacles,
  WaitForTick: WaitForTick,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  CMDConveyor: CMDConveyor,
  MultiPlayEventRequest: MultiPlayEventRequest,
  MultiEgoSetting: MultiEgoSetting,
  ShipState: ShipState,
  SetTrafficLight: SetTrafficLight,
  ObjectStatusExtended: ObjectStatusExtended,
  IntersectionStatus: IntersectionStatus,
  SkateboardStatus: SkateboardStatus,
  ScenarioLoad: ScenarioLoad,
  SyncModeCmdResponse: SyncModeCmdResponse,
  EventInfo: EventInfo,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  RadarDetections: RadarDetections,
  FaultInjection_Response: FaultInjection_Response,
  GhostMessage: GhostMessage,
  FaultStatusInfo: FaultStatusInfo,
  PREvent: PREvent,
  MoraiTLIndex: MoraiTLIndex,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  MultiPlayEventResponse: MultiPlayEventResponse,
  RobotState: RobotState,
  ObjectStatus: ObjectStatus,
  GetTrafficLightStatus: GetTrafficLightStatus,
  SyncModeCmd: SyncModeCmd,
  DdCtrlCmd: DdCtrlCmd,
  VelocityCmd: VelocityCmd,
  SyncModeInfo: SyncModeInfo,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  DillyCmd: DillyCmd,
  IntscnTL: IntscnTL,
  GVDirectCmd: GVDirectCmd,
  VehicleSpec: VehicleSpec,
  WoowaDillyStatus: WoowaDillyStatus,
  VehicleCollisionData: VehicleCollisionData,
  Lamps: Lamps,
  SaveSensorData: SaveSensorData,
};
