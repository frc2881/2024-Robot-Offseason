from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpimath import units
from extras.pathplannerlib.auto import AutoBuilder
from extras.pathplannerlib.path import PathPlannerPath
from extras.pathplannerlib.pathfinding import PathConstraints
if TYPE_CHECKING: from commands.game_commands import GameCommands
from classes import AutoPath, IntakeDirection
import constants

class AutoCommands:
  def __init__(
      self,
      gameCommands: "GameCommands"
    ) -> None:
    self._gameCommands = gameCommands

  def _getPath(self, autoPath: AutoPath) -> PathPlannerPath:
    return constants.Game.Auto.kPaths.get(autoPath)
  
  def _move(self, path: PathPlannerPath) -> Command:
    return AutoBuilder.pathfindThenFollowPath(
      path, constants.Subsystems.Drive.kPathFindingConstraints
    ).withName("AutoCommands:Move")
  
  def _pickup(self, path: PathPlannerPath) -> Command:
    return cmd.deadline(
      self._gameCommands.runIntakeCommand(IntakeDirection.Front),
      self._move(path)
    ).withTimeout(
      constants.Game.Auto.kPickupTimeout
    ).withName("AutoCommands:Pickup")
  
  def _score(self) -> Command:
    return cmd.deadline(
      self._gameCommands.launchToTargetCommand(),
      self._gameCommands.alignRobotToTargetCommand().withTimeout(constants.Game.Commands.kScoringAlignmentTimeout)
    ).withName("GameCommands:ScoreToTarget")
  
  def _scorePreload(self) -> Command:
    return cmd.sequence(
      self._gameCommands.launchAtPositionCommand(constants.Subsystems.Launcher.Arm.kPositionSubwoofer)
    ).withName("AutoCommands:ScorePreload")

  # ######################################################################
  # ################################ AUTOS ###############################
  # ######################################################################

  def test(self) -> Command:
    return cmd.sequence(
      AutoBuilder.pathfindThenFollowPath(
        constants.Game.Auto.kPaths.get(AutoPath.Test),
        PathConstraints(1.5, 1.5, units.degreesToRadians(270), units.degreesToRadians(360))
      )
    ).withName("AutoCommands:Test")

  def auto0(self) -> Command:
    return cmd.sequence(
      self._scorePreload()
    ).withName("AutoCommands:Auto0")
  
  # ############################################
  # ################ POSITION 1 ################
  # ############################################

  def auto10_1(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score()
    ).withName("AutoCommands:Auto10_1")
  
  def auto1_0_1(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1")
   
  def auto1_0_1_2_3(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup21)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup31)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1_2_3")
    
  def auto1_0_1_2_3_83(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup21)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup31)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1_2_3_83")
    
  def auto1_0_1_2_62(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup21)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup62)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1_2_3")
    
  def auto1_0_1_41(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup4)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1_41")
   
  def auto1_0_1_41_51(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup4)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup5)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1_41_51")
   
  def auto1_0_1_51(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup5)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1_51")
   
  def auto1_0_1_51_41(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup5)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup4)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1_51_41")
   
  def auto1_0_1_51_61(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup5)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup61)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1_51_61")
   
  def auto1_0_1_51_62(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup1)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup5)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup62)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1_51_62")
   
  # ############################################
  # ################ POSITION 2 ################
  # ############################################

  def auto20_2(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score()
    ).withName("AutoCommands:Auto20_2")
  
  def auto2_0_2(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score()
    ).withName("AutoCommands:Auto2_0_2")
   
  def auto2_0_2_1(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup13)),
      self._score()
    ).withName("AutoCommands:Auto2_0_2_1")
   
  def auto2_0_2_1_3(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup13)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup31)),
      self._score()
    ).withName("AutoCommands:Auto2_0_2_1_3")
   
  def auto2_0_2_3(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup31)),
      self._score()
    ).withName("AutoCommands:Auto2_0_2_3")
   
  def auto2_0_2_3_1(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup31)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup13)),
      self._score()
    ).withName("AutoCommands:Auto2_0_2_3_1")
   
  def auto2_0_2_62(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup62)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto2_0_2_62")
   
  def auto2_0_2_62_51(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup62)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup5)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score()
    ).withName("AutoCommands:Auto2_0_2_62_51")
   
  def auto2_0_2_62_72(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup62)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup72)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto2_0_2_62_72")
   
  def auto2_0_2_72(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup72)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto2_0_2_72")
   
  def auto2_0_2_72_62(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup2)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup72)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup62)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto2_0_2_72_62")
   
  # ############################################
  # ################ POSITION 3 ################
  # ############################################

  def auto30_3(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score()
    ).withName("AutoCommands:Auto30_3")
  
  def auto30_73(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(self._getPath(AutoPath.Pickup73)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score()
    ).withName("AutoCommands:Auto30_73")
   
  def auto30_83(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score()
    ).withName("AutoCommands:Auto30_83")
  
  def auto30_73_83(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(self._getPath(AutoPath.Pickup73)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score()
    ).withName("AutoCommands:Auto30_73_83")
   
  def auto30_83_73(self) -> Command:
    return cmd.sequence(
      self._scorePreload(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup73)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score()
    ).withName("AutoCommands:Auto30_83_73")
  
  def auto3_0_3(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3")
   
  def auto3_0_3_2_1(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup23)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup13)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_2_1")
   
  def auto3_0_3_2_1_41(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup23)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup13)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup4)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_2_1_41")
   
  def auto3_0_3_2_1_51(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup23)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup13)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup5)),
      self._move(self._getPath(AutoPath.ScoreStage1)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_2_1_51")
   
  def auto3_0_3_2_62(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(), 
      self._pickup(self._getPath(AutoPath.Pickup23)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup62)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto1_0_1_2_3")

  def auto3_0_3_62(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup63)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_62")
  
  def auto3_0_3_62_72(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup63)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup72)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_62_72")
  
  def auto3_0_3_72(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup73)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_72")
  
  def auto3_0_3_72_62(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup73)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup62)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_72_62")
  
  def auto3_0_3_73(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup73)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_73")
  
  def auto3_0_3_73_83(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup73)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_73_83")
  
  def auto3_0_3_82(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_82")
   
  def auto3_0_3_83(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_83")
   
  def auto3_0_3_82_62(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup62)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_82_62")
  
  def auto3_0_3_82_72(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup72)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_82_72")
  
  def auto3_0_3_83_62(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup62)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_83_62")
  
  def auto3_0_3_83_72(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup73)),
      self._move(self._getPath(AutoPath.ScoreStage2)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_83_72")
  
  def auto3_0_3_83_73(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.ScorePreload3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup8)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score(),
      self._pickup(self._getPath(AutoPath.Pickup73)),
      self._move(self._getPath(AutoPath.ScoreStage3)),
      self._score()
    ).withName("AutoCommands:Auto3_0_3_83_73")
  