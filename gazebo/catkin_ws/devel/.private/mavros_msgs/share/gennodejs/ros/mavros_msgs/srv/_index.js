
"use strict";

let FileOpen = require('./FileOpen.js')
let FileRead = require('./FileRead.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let FileTruncate = require('./FileTruncate.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let FileWrite = require('./FileWrite.js')
let WaypointPull = require('./WaypointPull.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let ParamPush = require('./ParamPush.js')
let FileRename = require('./FileRename.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let CommandTOL = require('./CommandTOL.js')
let WaypointPush = require('./WaypointPush.js')
let LogRequestList = require('./LogRequestList.js')
let ParamGet = require('./ParamGet.js')
let FileMakeDir = require('./FileMakeDir.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let CommandAck = require('./CommandAck.js')
let LogRequestData = require('./LogRequestData.js')
let CommandBool = require('./CommandBool.js')
let StreamRate = require('./StreamRate.js')
let ParamPull = require('./ParamPull.js')
let CommandLong = require('./CommandLong.js')
let MessageInterval = require('./MessageInterval.js')
let CommandInt = require('./CommandInt.js')
let CommandHome = require('./CommandHome.js')
let WaypointClear = require('./WaypointClear.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let SetMavFrame = require('./SetMavFrame.js')
let FileClose = require('./FileClose.js')
let FileChecksum = require('./FileChecksum.js')
let SetMode = require('./SetMode.js')
let FileRemove = require('./FileRemove.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let ParamSet = require('./ParamSet.js')
let FileList = require('./FileList.js')
let MountConfigure = require('./MountConfigure.js')

module.exports = {
  FileOpen: FileOpen,
  FileRead: FileRead,
  LogRequestEnd: LogRequestEnd,
  FileTruncate: FileTruncate,
  CommandVtolTransition: CommandVtolTransition,
  FileWrite: FileWrite,
  WaypointPull: WaypointPull,
  CommandTriggerInterval: CommandTriggerInterval,
  ParamPush: ParamPush,
  FileRename: FileRename,
  VehicleInfoGet: VehicleInfoGet,
  CommandTOL: CommandTOL,
  WaypointPush: WaypointPush,
  LogRequestList: LogRequestList,
  ParamGet: ParamGet,
  FileMakeDir: FileMakeDir,
  CommandTriggerControl: CommandTriggerControl,
  CommandAck: CommandAck,
  LogRequestData: LogRequestData,
  CommandBool: CommandBool,
  StreamRate: StreamRate,
  ParamPull: ParamPull,
  CommandLong: CommandLong,
  MessageInterval: MessageInterval,
  CommandInt: CommandInt,
  CommandHome: CommandHome,
  WaypointClear: WaypointClear,
  WaypointSetCurrent: WaypointSetCurrent,
  SetMavFrame: SetMavFrame,
  FileClose: FileClose,
  FileChecksum: FileChecksum,
  SetMode: SetMode,
  FileRemove: FileRemove,
  FileRemoveDir: FileRemoveDir,
  ParamSet: ParamSet,
  FileList: FileList,
  MountConfigure: MountConfigure,
};
