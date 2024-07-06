from typing import Any, Callable, TypeVar
import math
import numpy
import json
import time
import wpilib
import wpimath
from wpimath.geometry import Pose3d, Rotation2d
from wpilib import DriverStation
from rev import CANSparkBase, REVLibError
from lib import logger
from lib.classes import Alliance, RobotMode, RobotState
import robot

T = TypeVar("T")

def addRobotPeriodic(callback: Callable[[], None], period: float = 0.02, offset: float = 0) -> None:
  robot.Robot.getInstance().addPeriodic(callback, period, offset)

def getRobotState() -> RobotState:
  if wpilib.RobotState.isEnabled():
    return RobotState.Enabled
  elif wpilib.RobotState.isEStopped():
    return RobotState.EStopped
  else:
    return RobotState.Disabled

def getRobotMode() -> RobotMode:
  if wpilib.RobotState.isTeleop():
    return RobotMode.Teleop
  elif wpilib.RobotState.isAutonomous():
    return RobotMode.Auto
  elif wpilib.RobotState.isTest():
    return RobotMode.Test
  else:
    return RobotMode.Disabled

def getValueForRobotMode(autoValue: T, teleopValue: T) -> T:
  return autoValue if getRobotMode() == RobotMode.Auto else teleopValue 

def isAutonomousMode() -> bool:
  return getRobotMode() == RobotMode.Auto

def isCompetitionMode() -> bool:
  return DriverStation.getMatchTime() != -1

def getAlliance() -> Alliance:
  return Alliance(DriverStation.getAlliance() or Alliance.Blue)

def getValueForAlliance(blueValue: T, redValue: T) -> T:
  return blueValue if getAlliance() == Alliance.Blue else redValue

def getMatchTime() -> float:
  return DriverStation.getMatchTime()

def isValueInRange(value: float, minValue: float, maxValue: float) -> bool:
  return value >= minValue and value <= maxValue

def squareControllerInput(input: float, deadband: float) -> float:
  deadbandInput: float = wpimath.applyDeadband(input, deadband)
  return math.copysign(deadbandInput * deadbandInput, input)

def convertVoltsToPsi(sensorVoltage: float, supplyVoltage: float) -> float:
  return 250 * (sensorVoltage / supplyVoltage) - 25

def wrapAngle(angle: float) -> float:
  return wpimath.inputModulus(angle, -180, 180)

def getDistanceToPose(robotPose: Pose3d, targetPose: Pose3d) -> float:
  return robotPose.toPose2d().translation().distance(targetPose.toPose2d().translation())

def getHeadingToPose(robotPose: Pose3d, targetPose: Pose3d) -> float:
  translation = targetPose.toPose2d().relativeTo(robotPose.toPose2d()).translation()
  rotation = Rotation2d(translation.X(), translation.Y()).rotateBy(robotPose.toPose2d().rotation())
  return wrapAngle(rotation.degrees())

def getPitchToPose(robotPose: Pose3d, targetPose: Pose3d) -> float:
  return math.degrees(math.atan2((targetPose - robotPose).Z(), getDistanceToPose(robotPose, targetPose)))

def getInterpolatedValue(x: float, xs: list[float], ys: list[float]) -> float:
  try:
    return numpy.interp([x], xs, ys)[0]
  except:
    return math.nan

def enableSoftLimits(controller: CANSparkBase, isEnabled: bool) -> None:
  controller.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, isEnabled)
  controller.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, isEnabled)

def validateParam(error: REVLibError) -> None:
  time.sleep(0.001)
  if error != REVLibError.kOk:
    logger.error(f'REVLibError: {error}')

def toJson(value: Any) -> str:
  try:
    return json.dumps(value, default=lambda o: o.__dict__)
  except:
    return "{}"