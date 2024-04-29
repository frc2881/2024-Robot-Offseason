from enum import Enum, auto
from dataclasses import dataclass

class AutoPath(Enum):
  ScorePreload1 = auto()
  ScorePreload2 = auto()
  ScorePreload3 = auto()
  Pickup1 = auto()
  Pickup13 = auto()
  Pickup2 = auto()
  Pickup21 = auto()
  Pickup23 = auto()
  Pickup3 = auto()
  Pickup31 = auto()
  Pickup4 = auto()
  Pickup5 = auto()
  Pickup61 = auto()
  Pickup62 = auto()
  Pickup63 = auto()
  Pickup72 = auto()
  Pickup73 = auto()
  Pickup8 = auto()
  ScoreStage1 = auto()
  ScoreStage2 = auto()
  ScoreStage3 = auto()

class LightsMode(Enum):
  Default = auto()
  VisionNotReady = auto()
  IntakeReady = auto()
  IntakeNotReady = auto()
  LaunchReady = auto()

@dataclass
class LauncherArmPositionTarget:
  distance: float
  position: float

@dataclass
class LauncherRollersSpeeds:
  top: float
  bottom: float