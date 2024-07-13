from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import CANSparkLowLevel, CANSparkMax
from lib import utils, logger
from lib.classes import MotorDirection
from classes import IntakeDirection
import constants

class IntakeSubsystem(Subsystem):
  def __init__(
      self,
      getIntakeTargetDistance: Callable[[], units.millimeters],
      getLauncherTargetDistance: Callable[[], units.millimeters]
    ) -> None:
    super().__init__()
    self._getIntakeTargetDistance = getIntakeTargetDistance
    self._getLauncherTargetDistance = getLauncherTargetDistance

    self._constants = constants.Subsystems.Intake

    self._bottomBeltsMotor = CANSparkMax(self._constants.kBottomBeltsMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._bottomBeltsMotor.restoreFactoryDefaults())
    utils.validateParam(self._bottomBeltsMotor.setIdleMode(self._constants.kBottomBeltsMotorIdleMode))
    utils.validateParam(self._bottomBeltsMotor.setSmartCurrentLimit(self._constants.kBeltsMotorCurrentLimit))
    utils.validateParam(self._bottomBeltsMotor.setSecondaryCurrentLimit(self._constants.kBeltsMotorCurrentLimit))
    utils.validateParam(self._bottomBeltsMotor.burnFlash())

    self._topFrontBeltsMotor = CANSparkMax(self._constants.kTopFrontBeltsMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._topFrontBeltsMotor.restoreFactoryDefaults())
    utils.validateParam(self._topFrontBeltsMotor.setIdleMode(self._constants.kTopBeltsMotorIdleMode))
    utils.validateParam(self._topFrontBeltsMotor.setSmartCurrentLimit(self._constants.kBeltsMotorCurrentLimit))
    utils.validateParam(self._topFrontBeltsMotor.setSecondaryCurrentLimit(self._constants.kBeltsMotorCurrentLimit))
    utils.validateParam(self._topFrontBeltsMotor.burnFlash())

    self._topRearBeltsMotor = CANSparkMax(self._constants.kTopRearBeltsMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._topRearBeltsMotor.restoreFactoryDefaults())
    utils.validateParam(self._topRearBeltsMotor.setIdleMode(self._constants.kTopBeltsMotorIdleMode))
    utils.validateParam(self._topRearBeltsMotor.setSmartCurrentLimit(self._constants.kBeltsMotorCurrentLimit))
    utils.validateParam(self._topRearBeltsMotor.setSecondaryCurrentLimit(self._constants.kBeltsMotorCurrentLimit))
    utils.validateParam(self._topRearBeltsMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, intakeDirection: IntakeDirection) -> Command:
    return self.run(
      lambda: self._runBelts(MotorDirection.Reverse, MotorDirection.Forward, MotorDirection.Forward, self._constants.kBeltsSpeedIntake)
    ).until(
      lambda: self._getIntakeTargetDistance() <= self._constants.kIntakeTriggerDistanceRear
    ).onlyIf(
      lambda: intakeDirection == IntakeDirection.Rear
    ).andThen(
      self.run(
        lambda: self._runBelts(MotorDirection.Reverse, MotorDirection.Forward, MotorDirection.Forward, self._constants.kBeltsSpeedIntake)
      ).until(
        lambda: self._getIntakeTargetDistance() >= self._constants.kIntakeTriggerDistanceFront
      ).onlyIf(
        lambda: intakeDirection == IntakeDirection.Rear
      )
    ).andThen(
      self.run(
        lambda: self._runBelts(MotorDirection.Forward, MotorDirection.Reverse, MotorDirection.Forward, self._constants.kBeltsSpeedIntake)
      ).until(
        lambda: self._getLauncherTargetDistance() <= self._constants.kLauncherTriggerDistanceIntake
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Run")
  
  def ejectCommand(self) -> Command:
    return self.run(
      lambda: self._runBelts(MotorDirection.Reverse, MotorDirection.Forward, MotorDirection.Reverse, self._constants.kBeltsSpeedEject)
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Eject")
  
  def alignCommand(self) -> Command:
    return self.run(
      lambda: self._runBelts(MotorDirection.Reverse, MotorDirection.Forward, MotorDirection.Reverse, self._constants.kBeltsSpeedAlign)
    ).until(
      lambda: self._getLauncherTargetDistance() >= self._constants.kLauncherTriggerDistanceAlign
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Align")

  def launchCommand(self) -> Command:
    return self.run(
      lambda: self._runBelts(MotorDirection.Forward, MotorDirection.Reverse, MotorDirection.Forward, self._constants.kBeltsSpeedLaunch)
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Launch")

  def _runBelts(self, bottom: MotorDirection, topFront: MotorDirection, topRear: MotorDirection, speed: units.percent = 0) -> None:
    self._bottomBeltsMotor.set(self._getBeltsSpeed(speed, bottom))
    self._topFrontBeltsMotor.set(self._getBeltsSpeed(speed, topFront))
    self._topRearBeltsMotor.set(self._getBeltsSpeed(speed, topRear))

  def _getBeltsSpeed(self, speed: units.percent, motorDirection: MotorDirection) -> units.percent:
    return speed * (self._constants.kBeltsMotorMaxReverseOutput if motorDirection == MotorDirection.Reverse else self._constants.kBeltsMotorMaxForwardOutput)

  def isLaunchReady(self) -> bool:
    return utils.isValueInRange(self._getLauncherTargetDistance(), self._constants.kLauncherReadyDistanceMin, self._constants.kLauncherReadyDistanceMax)
  
  def reset(self) -> None:
    self._bottomBeltsMotor.set(0)
    self._topFrontBeltsMotor.set(0)
    self._topRearBeltsMotor.set(0)
    
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Intake/Belts/Speed", self._bottomBeltsMotor.get())
    SmartDashboard.putNumber("Robot/Intake/IsLaunchReady", self.isLaunchReady())
