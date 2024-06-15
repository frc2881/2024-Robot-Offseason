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
    self._topRollerMotor.setCANMaxRetries(10)
    utils.validateParam(self._topRollerMotor.restoreFactoryDefaults())
    utils.validateParam(self._topRollerMotor.setIdleMode(self._constants.kTopRollerMotorIdleMode))
    utils.validateParam(self._topRollerMotor.setSmartCurrentLimit(self._constants.kTopRollerMotorCurrentLimit))
    utils.validateParam(self._topRollerMotor.setSecondaryCurrentLimit(self._constants.kTopRollerMotorCurrentLimit))
    utils.validateParam(self._topRollerMotor.burnFlash())

    self._bottomRollerMotor = CANSparkFlex(self._constants.kBottomRollerMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._bottomRollerMotor.setCANMaxRetries(10)
    utils.validateParam(self._bottomRollerMotor.restoreFactoryDefaults())
    utils.validateParam(self._bottomRollerMotor.setIdleMode(self._constants.kBottomRollerMotorIdleMode))
    utils.validateParam(self._bottomRollerMotor.setSmartCurrentLimit(self._constants.kBottomRollerMotorCurrentLimit))
    utils.validateParam(self._bottomRollerMotor.setSecondaryCurrentLimit(self._constants.kBottomRollerMotorCurrentLimit))
    utils.validateParam(self._bottomRollerMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, rollersSpeeds: LauncherRollersSpeeds) -> Command:
    return self.startEnd(
      lambda: [
        self._topRollerMotor.set(rollersSpeeds.top * self._constants.kTopRollerMotorMaxReverseOutput),
        self._bottomRollerMotor.set(rollersSpeeds.bottom * self._constants.kBottomRollerMotorMaxForwardOutput)
      ],
      lambda: [
        self.reset()
      ]
    ).withName("LauncherRollersSubsystem:Run")

  def reset(self) -> None:
    self._topRollerMotor.set(0)
    self._bottomRollerMotor.set(0)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Launcher/Rollers/Top/Speed", self._topRollerMotor.get())
    SmartDashboard.putNumber("Robot/Launcher/Rollers/Bottom/Speed", self._bottomRollerMotor.get())