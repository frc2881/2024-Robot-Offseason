from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from rev import CANSparkLowLevel, CANSparkMax
from lib import utils, logger
from lib.classes import MotorDirection
from classes import IntakeDirection
import constants

class IntakeSubsystem(Subsystem):
  def __init__(
      self,
      getIntakeTargetDistance: Callable[[], float],
      getLauncherTargetDistance: Callable[[], float]
    ) -> None:
    super().__init__()
    self._getIntakeTargetDistance = getIntakeTargetDistance
    self._getLauncherTargetDistance = getLauncherTargetDistance

    self._constants = constants.Subsystems.Intake

    self._topFrontBeltsMotor = CANSparkMax(self._constants.kTopFrontBeltsMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._topFrontBeltsMotor.restoreFactoryDefaults())
    utils.validateParam(self._topFrontBeltsMotor.setIdleMode(self._constants.kTopFrontBeltsMotorIdleMode))
    utils.validateParam(self._topFrontBeltsMotor.setSmartCurrentLimit(self._constants.kTopFrontBeltsMotorCurrentLimit))
    utils.validateParam(self._topFrontBeltsMotor.setSecondaryCurrentLimit(self._constants.kTopFrontBeltsMotorCurrentLimit))
    utils.validateParam(self._topFrontBeltsMotor.burnFlash())

    self._topRearBeltsMotor = CANSparkMax(self._constants.kTopRearBeltsMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._topRearBeltsMotor.restoreFactoryDefaults())
    utils.validateParam(self._topRearBeltsMotor.setIdleMode(self._constants.kTopRearBeltsMotorIdleMode))
    utils.validateParam(self._topRearBeltsMotor.setSmartCurrentLimit(self._constants.kTopRearBeltsMotorCurrentLimit))
    utils.validateParam(self._topRearBeltsMotor.setSecondaryCurrentLimit(self._constants.kTopRearBeltsMotorCurrentLimit))
    utils.validateParam(self._topRearBeltsMotor.burnFlash())

    self._bottomBeltsMotor = CANSparkMax(self._constants.kBottomBeltsMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._bottomBeltsMotor.restoreFactoryDefaults())
    utils.validateParam(self._bottomBeltsMotor.setIdleMode(self._constants.kBottomBeltsMotorIdleMode))
    utils.validateParam(self._bottomBeltsMotor.setSmartCurrentLimit(self._constants.kBottomBeltsMotorCurrentLimit))
    utils.validateParam(self._bottomBeltsMotor.setSecondaryCurrentLimit(self._constants.kBottomBeltsMotorCurrentLimit))
    utils.validateParam(self._bottomBeltsMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, intakeDirection: IntakeDirection) -> Command:
    return self.run(
      lambda: [
        self._runTopFrontBelts(MotorDirection.Forward, self._constants.kBeltsSpeedIntake),
        self._runTopRearBelts(MotorDirection.Forward, self._constants.kBeltsSpeedIntake),
        self._runBottomBelts(MotorDirection.Reverse, self._constants.kBeltsSpeedIntake)
      ]
    ).until(
      lambda: self._getIntakeTargetDistance() <= self._constants.kIntakeTriggerDistanceIn
    ).onlyIf(
      lambda: intakeDirection == IntakeDirection.Rear
    ).andThen(
      self.run(
        lambda: [
          self._runTopFrontBelts(MotorDirection.Forward, self._constants.kBeltsSpeedIntake),
          self._runTopRearBelts(MotorDirection.Forward, self._constants.kBeltsSpeedIntake),
          self._runBottomBelts(MotorDirection.Reverse, self._constants.kBeltsSpeedIntake)
        ]
      ).until(
        lambda: self._getIntakeTargetDistance() >= self._constants.kIntakeTriggerDistanceOut
      ).onlyIf(
        lambda: intakeDirection == IntakeDirection.Rear
      )
    ).andThen(
      self.run(
        lambda: [
          self._runTopFrontBelts(MotorDirection.Reverse, self._constants.kBeltsSpeedIntake),
          self._runTopRearBelts(MotorDirection.Forward, self._constants.kBeltsSpeedIntake),
          self._runBottomBelts(MotorDirection.Forward, self._constants.kBeltsSpeedIntake)
        ]
      ).until(
        lambda: self._getLauncherTargetDistance() <= self._constants.kLauncherTriggerDistanceIn
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Run")
  
  def alignCommand(self) -> Command:
    return self.run(
      lambda: [
        self._runTopFrontBelts(MotorDirection.Forward, self._constants.kBeltsSpeedAlign),
        self._runTopRearBelts(MotorDirection.Reverse, self._constants.kBeltsSpeedAlign),
        self._runBottomBelts(MotorDirection.Reverse, self._constants.kBeltsSpeedAlign)
      ]
    ).until(
      lambda: self._getLauncherTargetDistance() >= self._constants.kLauncherTargetDistanceMax
    ).andThen(
      self.run(
        lambda: [
          self._runTopFrontBelts(MotorDirection.Reverse, self._constants.kBeltsSpeedAlign),
          self._runTopRearBelts(MotorDirection.Forward, self._constants.kBeltsSpeedAlign),
          self._runBottomBelts(MotorDirection.Forward, self._constants.kBeltsSpeedAlign)
        ]
      ).until(
        lambda: self._getLauncherTargetDistance() <= self._constants.kLauncherTriggerDistanceIn
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Align")
  
  def ejectCommand(self) -> Command:
    return self.run(
      lambda: [
        self._runTopFrontBelts(MotorDirection.Forward, self._constants.kBeltsSpeedEject),
        self._runTopRearBelts(MotorDirection.Reverse, self._constants.kBeltsSpeedEject),
        self._runBottomBelts(MotorDirection.Reverse, self._constants.kBeltsSpeedEject)
      ]
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Eject")

  def launchCommand(self) -> Command:
    return self.run(
      lambda: [
        self._runTopFrontBelts(MotorDirection.Reverse, self._constants.kBeltsSpeedLaunch),
        self._runTopRearBelts(MotorDirection.Forward, self._constants.kBeltsSpeedLaunch),
        self._runBottomBelts(MotorDirection.Forward, self._constants.kBeltsSpeedLaunch)
      ]
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Launch")

  def _runTopFrontBelts(self, motorDirection: MotorDirection, speed: float = 1.0) -> None:
    match motorDirection:
      case MotorDirection.Forward:
        speed *= self._constants.kTopFrontBeltsMotorMaxForwardOutput
      case MotorDirection.Reverse:
        speed *= self._constants.kTopFrontBeltsMotorMaxReverseOutput
      case MotorDirection.Stopped:
        speed = 0
    self._topFrontBeltsMotor.set(speed)

  def _runTopRearBelts(self, motorDirection: MotorDirection, speed: float = 1.0) -> None:
    match motorDirection:
      case MotorDirection.Forward:
        speed *= self._constants.kTopRearBeltsMotorMaxForwardOutput
      case MotorDirection.Reverse:
        speed *= self._constants.kTopRearBeltsMotorMaxReverseOutput
      case MotorDirection.Stopped:
        speed = 0
    self._topRearBeltsMotor.set(speed)

  def _runBottomBelts(self, motorDirection: MotorDirection, speed: float = 1.0) -> None:
    match motorDirection:
      case MotorDirection.Forward:
        speed *= self._constants.kBottomBeltsMotorMaxForwardOutput
      case MotorDirection.Reverse:
        speed *= self._constants.kBottomBeltsMotorMaxReverseOutput
      case MotorDirection.Stopped:
        speed = 0
    self._bottomBeltsMotor.set(speed)

  def isLaunchReady(self) -> bool:
    return utils.isValueInRange(self._getLauncherTargetDistance(), self._constants.kLauncherTargetDistanceMin, self._constants.kLauncherTargetDistanceMax)
  
  def reset(self) -> None:
    self._runTopFrontBelts(MotorDirection.Stopped)
    self._runTopRearBelts(MotorDirection.Stopped)
    self._runBottomBelts(MotorDirection.Stopped)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Intake/IsLaunchReady", self.isLaunchReady())
    SmartDashboard.putNumber("Robot/Intake/Belts/Speed", self._bottomBeltsMotor.get())
