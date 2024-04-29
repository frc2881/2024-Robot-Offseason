from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from rev import CANSparkLowLevel, CANSparkMax
from lib import utils, logger
from lib.classes import MotorDirection
import constants

class IntakeSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake

    self._topBeltsMotor = CANSparkMax(self._constants.kTopBeltsMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._topBeltsMotor.restoreFactoryDefaults())
    utils.validateParam(self._topBeltsMotor.setIdleMode(self._constants.kTopBeltsMotorIdleMode))
    utils.validateParam(self._topBeltsMotor.setSmartCurrentLimit(self._constants.kTopBeltsMotorCurrentLimit))
    utils.validateParam(self._topBeltsMotor.setSecondaryCurrentLimit(self._constants.kTopBeltsMotorCurrentLimit))
    utils.validateParam(self._topBeltsMotor.burnFlash())

    self._bottomBeltsMotor = CANSparkMax(self._constants.kBottomBeltsMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._bottomBeltsMotor.restoreFactoryDefaults())
    utils.validateParam(self._bottomBeltsMotor.setIdleMode(self._constants.kBottomBeltsMotorIdleMode))
    utils.validateParam(self._bottomBeltsMotor.setSmartCurrentLimit(self._constants.kBottomBeltsMotorCurrentLimit))
    utils.validateParam(self._bottomBeltsMotor.setSecondaryCurrentLimit(self._constants.kBottomBeltsMotorCurrentLimit))
    utils.validateParam(self._bottomBeltsMotor.burnFlash())

    self._rollersMotor = CANSparkMax(self._constants.kRollersMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._rollersMotor.restoreFactoryDefaults())
    utils.validateParam(self._rollersMotor.setIdleMode(self._constants.kRollersMotorIdleMode))
    utils.validateParam(self._rollersMotor.setSmartCurrentLimit(self._constants.kRollersMotorCurrentLimit))
    utils.validateParam(self._rollersMotor.setSecondaryCurrentLimit(self._constants.kRollersMotorCurrentLimit))
    utils.validateParam(self._rollersMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, getLauncherBottomHasTarget: Callable[[], bool]) -> Command:
    return self.startEnd(
      lambda: [
        self._runTopBelts(
          MotorDirection.Forward, 
          utils.getValueForRobotMode(
            self._constants.kIntakeBeltsAutoSpeed, 
            self._constants.kIntakeBeltsSpeed
          )
        ),
        self._runBottomBelts(
          MotorDirection.Forward,
          utils.getValueForRobotMode(
            self._constants.kIntakeBeltsAutoSpeed, 
            self._constants.kIntakeBeltsSpeed
          )
        ),
        self._runRollers(MotorDirection.Reverse)
      ], 
      lambda: None
    )\
    .until(
      lambda: getLauncherBottomHasTarget()
    ).andThen(
      cmd.waitSeconds(
        utils.getValueForRobotMode(self._constants.kIntakeCompletionAutoDelay, self._constants.kIntakeCompletionDelay)
      )
    ).finallyDo(
      lambda end: [
        self._runTopBelts(MotorDirection.Stopped),
        self._runBottomBelts(MotorDirection.Stopped),
        self._runRollers(MotorDirection.Stopped)
      ]
    )

  def adjustPositionCommand(self, getLauncherTopHasTarget: Callable[[], bool]) -> Command:
    return self.startEnd(
      lambda: self._runTopBelts(MotorDirection.Reverse, self._constants.kIntakeBeltsAdjustPositionSpeed), 
      lambda: self._runTopBelts(MotorDirection.Stopped)
    ).until(
      lambda: not getLauncherTopHasTarget()
    )

  def ejectCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._runTopBelts(MotorDirection.Reverse),
        self._runBottomBelts(MotorDirection.Reverse),
        self._runRollers(MotorDirection.Forward)
      ],
      lambda: [
        self._runTopBelts(MotorDirection.Stopped),
        self._runBottomBelts(MotorDirection.Stopped),
        self._runRollers(MotorDirection.Stopped)
      ]
    )

  def launchCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._runTopBelts(MotorDirection.Forward),
        self._runBottomBelts(MotorDirection.Forward),
        self._runRollers(MotorDirection.Stopped)
      ],
      lambda: [
        self._runTopBelts(MotorDirection.Stopped),
        self._runBottomBelts(MotorDirection.Stopped),
        self._runRollers(MotorDirection.Stopped)
      ]
    )

  def _runTopBelts(self, motorDirection: MotorDirection, speed: float = 1.0) -> None:
    match motorDirection:
      case MotorDirection.Forward:
        speed *= self._constants.kTopBeltsMotorMaxForwardOutput
      case MotorDirection.Reverse:
        speed *= self._constants.kTopBeltsMotorMaxReverseOutput
      case MotorDirection.Stopped:
        speed = 0
    self._topBeltsMotor.set(speed)

  def _runBottomBelts(self, motorDirection: MotorDirection, speed: float = 1.0) -> None:
    match motorDirection:
      case MotorDirection.Forward:
        speed *= self._constants.kBottomBeltsMotorMaxForwardOutput
      case MotorDirection.Reverse:
        speed *= self._constants.kBottomBeltsMotorMaxReverseOutput
      case MotorDirection.Stopped:
        speed = 0
    self._bottomBeltsMotor.set(speed)

  def _runRollers(self, motorDirection: MotorDirection, speed: float = 1.0) -> None:
    match motorDirection:
      case MotorDirection.Forward:
        speed *= self._constants.kRollersMotorMaxForwardOutput
      case MotorDirection.Reverse:
        speed *= self._constants.kRollersMotorMaxReverseOutput
      case MotorDirection.Stopped:
        speed = 0
    self._rollersMotor.set(speed)

  def reset(self) -> None:
    self._topBeltsMotor.set(0.0)
    self._bottomBeltsMotor.set(0.0)
    self._rollersMotor.set(0.0)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Intake/Belt/Top/Speed", self._topBeltsMotor.get())
    SmartDashboard.putNumber("Robot/Intake/Belt/Bottom/Speed", self._bottomBeltsMotor.get())
    SmartDashboard.putNumber("Robot/Intake/Roller/Speed", self._rollersMotor.get())