from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from rev import CANSparkLowLevel, CANSparkFlex
from lib import utils, logger
from classes import LauncherRollersSpeeds
import constants

class LauncherRollersSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    
    self._constants = constants.Subsystems.Launcher

    self._topRollerMotor = CANSparkFlex(self._constants.kTopRollerMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._topRollerMotorEncoder = self._topRollerMotor.getEncoder()
    self._topRollerMotor.setCANMaxRetries(10)
    utils.validateParam(self._topRollerMotor.restoreFactoryDefaults())
    utils.validateParam(self._topRollerMotor.setIdleMode(self._constants.kTopRollerMotorIdleMode))
    utils.validateParam(self._topRollerMotor.setSmartCurrentLimit(self._constants.kTopRollerMotorCurrentLimit))
    utils.validateParam(self._topRollerMotor.setSecondaryCurrentLimit(self._constants.kTopRollerMotorCurrentLimit))
    self._topRollerMotor.setInverted(True)
    utils.validateParam(self._topRollerMotor.burnFlash())
    
    self._bottomRollerMotor = CANSparkFlex(self._constants.kBottomRollerMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._bottomRollerMotorEncoder = self._bottomRollerMotor.getEncoder()
    self._bottomRollerMotor.setCANMaxRetries(10)
    utils.validateParam(self._bottomRollerMotor.restoreFactoryDefaults())
    utils.validateParam(self._bottomRollerMotor.setIdleMode(self._constants.kBottomRollerMotorIdleMode))
    utils.validateParam(self._bottomRollerMotor.setSmartCurrentLimit(self._constants.kBottomRollerMotorCurrentLimit))
    utils.validateParam(self._bottomRollerMotor.setSecondaryCurrentLimit(self._constants.kBottomRollerMotorCurrentLimit))
    utils.validateParam(self._bottomRollerMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, rollersSpeeds: LauncherRollersSpeeds) -> Command:
    return self.run(
      lambda: [
        self._topRollerMotor.set(rollersSpeeds.top * self._constants.kTopRollerMotorMaxForwardOutput),
        self._bottomRollerMotor.set(rollersSpeeds.bottom * self._constants.kBottomRollerMotorMaxForwardOutput)
      ]
    ).finallyDo(
      lambda end: self.reset()
    ).withName("LauncherRollersSubsystem:Run")
  
  def isLaunchReady(self, rollerSpeeds: LauncherRollersSpeeds) -> bool:
    return (
      self._topRollerMotorEncoder.getVelocity() / self._constants.kRollerMotorFreeSpeedRpm >= rollerSpeeds.top * self._constants.kRollerSpeedsTargetThreshold
      and 
      self._bottomRollerMotorEncoder.getVelocity() / self._constants.kRollerMotorFreeSpeedRpm >= rollerSpeeds.bottom * self._constants.kRollerSpeedsTargetThreshold
    )

  def reset(self) -> None:
    self._topRollerMotor.set(0)
    self._bottomRollerMotor.set(0)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Launcher/Rollers/Top/Speed/Target", self._topRollerMotor.get())
    SmartDashboard.putNumber("Robot/Launcher/Rollers/Top/Speed/Actual", self._topRollerMotorEncoder.getVelocity())
    SmartDashboard.putNumber("Robot/Launcher/Rollers/Bottom/Speed/Target", self._bottomRollerMotor.get())
    SmartDashboard.putNumber("Robot/Launcher/Rollers/Bottom/Velocity/Actual", self._bottomRollerMotorEncoder.getVelocity())
