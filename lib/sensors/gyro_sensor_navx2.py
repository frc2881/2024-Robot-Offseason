from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import SmartDashboard, RobotBase
from wpimath.geometry import Rotation2d, Pose2d
import navx
from lib import utils
if TYPE_CHECKING: import constants

class GyroSensor():
  def __init__(
      self,
      constants: "constants.Sensors.Gyro"
    ) -> None:
    self._gyro = navx.AHRS(constants.kSerialPort)
    self._constants = constants

    self._baseKey = f'Robot/Sensor/Gyro'

    utils.addRobotPeriodic(self._updateTelemetry)
  
  def _calibrate(self) -> None:
    if RobotBase.isReal():
      self._gyro.calibrate()

  def _reset(self, heading: float) -> None:
    self._gyro.reset()
    self._gyro.setAngleAdjustment(heading)
  
  def getHeading(self) -> float:
    return utils.wrapAngle(self._gyro.getAngle())
  
  def getRotation(self) -> float:
    return Rotation2d.fromDegrees(self.getHeading())
  
  def getPitch(self) -> float:
    return self._gyro.getPitch()
  
  def getRoll(self) -> float:
    return self._gyro.getRoll()
  
  def getTurnRate(self) -> float:
    return self._gyro.getRate()
  
  def resetCommand(self) -> Command:
    return cmd.runOnce(
      lambda: self._reset(0)
    ).ignoringDisable(True).withName("GyroSensor:Reset")
  
  def calibrateCommand(self) -> Command:
    return cmd.sequence(
      cmd.runOnce(
        lambda: [
          SmartDashboard.putBoolean(f'{self._baseKey}/IsCalibrating', True),
          self._calibrate()
        ]
      ),
      cmd.waitSeconds(self._constants.kCalibrationWaitTime),
      cmd.runOnce(
        lambda: SmartDashboard.putBoolean(f'{self._baseKey}/IsCalibrating', False)
      )
    ).ignoringDisable(True).withName("GyroSensor:Calibrate")
  
  def alignRobotToField(self, robotPose: Pose2d) -> None:
    self._reset(utils.wrapAngle(robotPose.rotation().degrees() + utils.getValueForAlliance(0.0, 180.0)))
 
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Heading', self.getHeading())
    SmartDashboard.putNumber(f'{self._baseKey}/Pitch', self.getPitch())
    SmartDashboard.putNumber(f'{self._baseKey}/Roll', self.getRoll())
    SmartDashboard.putNumber(f'{self._baseKey}/TurnRate', self.getTurnRate())