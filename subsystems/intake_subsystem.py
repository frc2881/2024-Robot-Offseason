from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from rev import CANSparkLowLevel, CANSparkMax
from lib import utils, logger
from lib.classes import MotorDirection
from classes import IntakeDirection
import constants

class IntakeSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
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

  def runCommand(
    self, 
    intakeDirection: IntakeDirection, 
    getIntakeTargetDistance: Callable[[], float], 
    getLauncherTargetDistance: Callable[[], float]
  ) -> Command:
    return self.run(
      lambda: [
        self._runTopFrontBelts(MotorDirection.Forward, self._constants.kBeltsDefaultSpeed),
        self._runTopRearBelts(MotorDirection.Forward, self._constants.kBeltsDefaultSpeed),
        self._runBottomBelts(MotorDirection.Reverse, self._constants.kBeltsDefaultSpeed)
      ]
    ).until(
      lambda: getIntakeTargetDistance() <= self._constants.kIntakeTriggerDistanceIn
    ).onlyIf(
      lambda: intakeDirection == IntakeDirection.Rear
    ).andThen(
      self.run(
        lambda: [
          self._runTopFrontBelts(MotorDirection.Forward, self._constants.kBeltsDefaultSpeed),
          self._runTopRearBelts(MotorDirection.Forward, self._constants.kBeltsDefaultSpeed),
          self._runBottomBelts(MotorDirection.Reverse, self._constants.kBeltsDefaultSpeed)
        ]
      ).until(
        lambda: getIntakeTargetDistance() >= self._constants.kIntakeTriggerDistanceOut
      ).onlyIf(
        lambda: intakeDirection == IntakeDirection.Rear
      )
    ).andThen(
      self.run(
        lambda: [
          self._runTopFrontBelts(MotorDirection.Reverse, self._constants.kBeltsDefaultSpeed),
          self._runTopRearBelts(MotorDirection.Forward, self._constants.kBeltsDefaultSpeed),
          self._runBottomBelts(MotorDirection.Forward, self._constants.kBeltsDefaultSpeed)
        ]
      ).until(
        lambda: getLauncherTargetDistance() <= self._constants.kLauncherTriggerDistanceIn
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Run")

  def alignCommand(self, getLauncherTargetDistance: Callable[[], float]) -> Command:
    return self.startEnd(
      lambda: [
        self._runTopFrontBelts(MotorDirection.Forward, self._constants.kBeltsAlignmentSpeed),
        self._runTopRearBelts(MotorDirection.Reverse, self._constants.kBeltsAlignmentSpeed),
      ], 
      lambda: self.reset()
    ).until(
      lambda: getLauncherTargetDistance() >= self._constants.kLauncherTargetDistanceMin
    ).withName("IntakeSubsystem:Align")

  def ejectCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._runTopFrontBelts(MotorDirection.Forward),
        self._runTopRearBelts(MotorDirection.Reverse),
        self._runBottomBelts(MotorDirection.Reverse)
      ],
      lambda: self.reset()
    ).withName("IntakeSubsystem:Eject")

  def launchCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._runTopFrontBelts(MotorDirection.Reverse),
        self._runTopRearBelts(MotorDirection.Forward),
        self._runBottomBelts(MotorDirection.Forward)
      ],
      lambda: self.reset()
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

  def reset(self) -> None:
    self._runTopFrontBelts(MotorDirection.Stopped)
    self._runTopRearBelts(MotorDirection.Stopped)
    self._runBottomBelts(MotorDirection.Stopped)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Intake/Belts/TopFront/Speed", self._topFrontBeltsMotor.get())
    SmartDashboard.putNumber("Robot/Intake/Belts/TopRear/Speed", self._topRearBeltsMotor.get())
    SmartDashboard.putNumber("Robot/Intake/Belts/Bottom/Speed", self._bottomBeltsMotor.get())
