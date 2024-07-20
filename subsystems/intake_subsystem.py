from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command, cmd
from rev import CANSparkBase, CANSparkLowLevel, CANSparkMax
from lib import utils, logger
from lib.classes import MotorDirection
from classes import IntakeDirection
import constants

class IntakeSubsystem(Subsystem):
  def __init__(
      self,
      getIntakeHasTarget: Callable[[], bool],
      getIntakeDistance: Callable[[], units.millimeters],
      getLauncherHasTarget: Callable[[], bool],
      getLauncherDistance: Callable[[], units.millimeters]
    ) -> None:
    super().__init__()
    self._getIntakeHasTarget = getIntakeHasTarget
    self._getIntakeDistance = getIntakeDistance
    self._getLauncherHasTarget = getLauncherHasTarget
    self._getLauncherDistance = getLauncherDistance

    self._constants = constants.Subsystems.Intake

    self._bottomMotor = CANSparkMax(self._constants.kBottomMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._bottomMotor.restoreFactoryDefaults())
    utils.validateParam(self._bottomMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._bottomMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._bottomMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    self._bottomMotor.setInverted(True)
    utils.validateParam(self._bottomMotor.burnFlash())

    self._topFrontMotor = CANSparkMax(self._constants.kTopFrontMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._topFrontMotor.restoreFactoryDefaults())
    utils.validateParam(self._topFrontMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._topFrontMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._topFrontMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._topFrontMotor.burnFlash())

    self._topRearMotor = CANSparkMax(self._constants.kTopRearMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    utils.validateParam(self._topRearMotor.restoreFactoryDefaults())
    utils.validateParam(self._topRearMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._topRearMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._topRearMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._topRearMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, intakeDirection: IntakeDirection) -> Command:
    return self.run(
      lambda: self._run(
        MotorDirection.Forward, 
        MotorDirection.Forward, 
        MotorDirection.Forward, 
        self._constants.kSpeedIntake
      )
    ).until(
      lambda: self._getIntakeHasTarget()
    ).onlyIf(
      lambda: intakeDirection == IntakeDirection.Rear
    ).andThen(
      self.run(
        lambda: self._run(
          MotorDirection.Forward, 
          MotorDirection.Forward, 
          MotorDirection.Forward, 
          self._constants.kSpeedIntake
        )
      ).until(
        lambda: not self._getIntakeHasTarget()
      ).onlyIf(
        lambda: intakeDirection == IntakeDirection.Rear
      )
    ).andThen(
      self.run(
        lambda: self._run(
          MotorDirection.Reverse, 
          MotorDirection.Reverse, 
          MotorDirection.Forward, 
          self._constants.kSpeedIntake
        )
      ).until(
        lambda: self._getLauncherHasTarget() and self._getLauncherDistance() <= self._constants.kDistanceLauncherIntake
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Run")
  
  def reloadCommand(self) -> Command:
    return self.ejectCommand().withTimeout(
      constants.Subsystems.Intake.kReloadTimeout
    ).andThen(
      self.runCommand(IntakeDirection.Front)
    ).withName("IntakeSubsystem:Reload")

  def ejectCommand(self) -> Command:
    return self.run(
      lambda: self._run(
        MotorDirection.Forward, 
        MotorDirection.Forward, 
        MotorDirection.Reverse, 
        self._constants.kSpeedEject
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Eject")
  
  def alignCommand(self) -> Command:
    return cmd.run(
      lambda: self._run(
        MotorDirection.Forward, 
        MotorDirection.Forward, 
        MotorDirection.Reverse, 
        self._constants.kSpeedAlign
      )
    ).withTimeout(
      self._constants.kAlignTimeout
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Align")

  def launchCommand(self) -> Command:
    return self.run(
      lambda: self._run(
        MotorDirection.Reverse, 
        MotorDirection.Reverse, 
        MotorDirection.Forward, 
        self._constants.kSpeedLaunch
      )
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:Launch")

  def _run(self, bottom: MotorDirection, topFront: MotorDirection, topRear: MotorDirection, speed: units.percent = 0) -> None:
    self._bottomMotor.set(self._getSpeed(speed, bottom))
    self._topFrontMotor.set(self._getSpeed(speed, topFront))
    self._topRearMotor.set(self._getSpeed(speed, topRear))

  def _getSpeed(self, speed: units.percent, motorDirection: MotorDirection) -> units.percent:
    if motorDirection != MotorDirection.Stopped:
      return speed * (self._constants.kMotorMaxReverseOutput if motorDirection == MotorDirection.Reverse else self._constants.kMotorMaxForwardOutput)
    else:
      return 0

  def isLoaded(self) -> bool:
    return self._getLauncherHasTarget() and self._getLauncherDistance() <= self._constants.kDistanceLauncherReadyMax

  def isLaunchReady(self) -> bool:
    return self._getLauncherHasTarget() and utils.isValueInRange(self._getLauncherDistance(), self._constants.kDistanceLauncherReadyMin, self._constants.kDistanceLauncherReadyMax)
  
  def reset(self) -> None:
    self._bottomMotor.set(0)
    self._topFrontMotor.set(0)
    self._topRearMotor.set(0)
    
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Intake/Speed", self._topFrontMotor.get())
    SmartDashboard.putBoolean("Robot/Intake/IsLoaded", self.isLoaded())
    SmartDashboard.putBoolean("Robot/Intake/IsLaunchReady", self.isLaunchReady())
