import math
from commands2 import Command, cmd
from commands2.button import CommandXboxController, Trigger
from wpilib import XboxController
from lib import utils
from lib.classes import ControllerRumblePattern

class GameController(CommandXboxController):
  def __init__(
      self, 
      port: int, 
      inputDeadband: float
    ) -> None:
    super().__init__(port)
    self._inputDeadband = inputDeadband

  def getLeftY(self) -> float:
    return utils.squareControllerInput(-super().getLeftY(), self._inputDeadband)
  
  def leftY(self) -> Trigger:
    return Trigger(lambda: math.fabs(super().getLeftY()) > self._inputDeadband)
  
  def getLeftX(self) -> float:
    return utils.squareControllerInput(-super().getLeftX(), self._inputDeadband)
  
  def leftX(self) -> Trigger:
    return Trigger(lambda: math.fabs(super().getLeftX()) > self._inputDeadband)
  
  def getRightY(self) -> float:
    return utils.squareControllerInput(-super().getRightY(), self._inputDeadband)
  
  def rightY(self) -> Trigger:
    return Trigger(lambda: math.fabs(super().getRightY()) > self._inputDeadband)
  
  def getRightX(self) -> float:
    return utils.squareControllerInput(-super().getRightX(), self._inputDeadband)
  
  def rightX(self) -> Trigger:
    return Trigger(lambda: math.fabs(super().getRightX()) > self._inputDeadband)
  
  def rumbleCommand(self, pattern: ControllerRumblePattern) -> Command:
    match pattern:
      case ControllerRumblePattern.Short:
        return cmd.run(
          lambda: self._hid.setRumble(XboxController.RumbleType.kBothRumble, 1)
        ).withTimeout(
          0.5
        ).finallyDo(
          lambda end: self._hid.setRumble(XboxController.RumbleType.kBothRumble, 0)
        )
      case ControllerRumblePattern.Long:
        return cmd.run(
          lambda: self._hid.setRumble(XboxController.RumbleType.kBothRumble, 1)
        ).withTimeout(
          1.5
        ).finallyDo(
          lambda end: self._hid.setRumble(XboxController.RumbleType.kBothRumble, 0)
        )
      case _:
        return cmd.none()