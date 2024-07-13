from typing import TYPE_CHECKING
from wpilib import RobotBase
from commands2 import Command, cmd
if TYPE_CHECKING: from robot_container import RobotContainer
from lib import utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
from classes import IntakeDirection, LauncherRollersSpeeds
import constants

class GameCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self.robot = robot

  def runIntakeCommand(self, intakeDirection: IntakeDirection) -> Command:
    return cmd.sequence(
      self.robot.intakeSubsystem.runCommand(
        intakeDirection
      ).deadlineWith(
        self.robot.launcherArmSubsystem.alignToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionIntake)
      ),
      self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
    ).withName("GameCommands:RunIntake")
  
  def reloadIntakeCommand(self) -> Command:
    return cmd.sequence(
      self.robot.intakeSubsystem.ejectCommand().withTimeout(constants.Subsystems.Intake.kReloadTimeout),
      self.robot.intakeSubsystem.runCommand(IntakeDirection.Front),
      self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
    ).withName("GameCommands:ReloadIntake")

  def ejectIntakeCommand(self) -> Command:
    return cmd.sequence(
      self.robot.intakeSubsystem.ejectCommand()
    ).withName("GameCommands:EjectIntake")
  
  def alignRobotToTargetCommand(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self.robot.driveSubsystem.alignToTargetCommand(
          lambda: self.robot.localizationSubsystem.getPose(), 
          lambda: self.robot.localizationSubsystem.getTargetHeading()
        ),
        self.rumbleControllersCommand(ControllerRumbleMode.Operator, ControllerRumblePattern.Short)
      ),
      self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
    ).withName("GameCommands:AlignRobotToTarget")

  def alignLauncherToTargetCommand(self) -> Command:
    return cmd.sequence(
      self.robot.intakeSubsystem.alignCommand(),
      cmd.parallel(
        self.robot.launcherArmSubsystem.alignToTargetCommand(lambda: self.robot.localizationSubsystem.getTargetDistance()),
        self.robot.launcherRollersSubsystem.runCommand(constants.Subsystems.Launcher.Rollers.kSpeedsDefault)
      )
    ).withName("GameCommands:AlignLauncherToTarget")

  def alignLauncherToPositionCommand(self, position: float, launcherRollerSpeeds = constants.Subsystems.Launcher.Rollers.kSpeedsDefault) -> Command:
    return cmd.sequence(
      self.robot.intakeSubsystem.alignCommand(),
      cmd.parallel(
        self.robot.launcherArmSubsystem.alignToPositionCommand(position),
        self.robot.launcherRollersSubsystem.runCommand(launcherRollerSpeeds)
      )
    ).withName("GameCommands:AlignLauncherToPosition")
  
  def runLauncherCommand(self, launcherRollerSpeeds = constants.Subsystems.Launcher.Rollers.kSpeedsDefault, position: float = 0) -> Command:
    return cmd.parallel(
      self.robot.launcherRollersSubsystem.runCommand(launcherRollerSpeeds),
      cmd.either(
        self.robot.launcherArmSubsystem.alignToPositionCommand(position),
        self.robot.launcherArmSubsystem.alignToTargetCommand(lambda: self.robot.localizationSubsystem.getTargetDistance()),
        lambda: position > 0
      ),
      cmd.sequence(
        cmd.waitUntil(
          lambda: self.robot.launcherRollersSubsystem.isLaunchReady()
        ).withTimeout(constants.Game.Commands.kScoringLaunchTimeout),
        self.robot.intakeSubsystem.launchCommand()
      )
    ).onlyIf(
      lambda: self.robot.launcherDistanceSensor.hasTarget()
    ).until(
      lambda: not self.robot.launcherDistanceSensor.hasTarget()
    ).finallyDo(lambda end: [
      self.robot.driveSubsystem.clearTargetAlignment(),
      self.robot.launcherArmSubsystem.clearTargetAlignment()
    ]).withName("GameCommands:RunLauncher")

  def launchToTargetCommand(self) -> Command:
    return cmd.sequence(
      self.alignLauncherToTargetCommand().withTimeout(constants.Game.Commands.kScoringAlignmentTimeout),
      self.runLauncherCommand()
    ).withName("GameCommands:LaunchToTarget")
  
  def launchAtPositionCommand(self, position: float, launcherRollerSpeeds = constants.Subsystems.Launcher.Rollers.kSpeedsDefault) -> Command:
    return cmd.sequence(
      self.alignLauncherToPositionCommand(position, launcherRollerSpeeds).withTimeout(constants.Game.Commands.kScoringAlignmentTimeout),
      self.runLauncherCommand(launcherRollerSpeeds, position)
    ).withName("GameCommands:LaunchAtPosition")
  
  def scoreToTargetCommand(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self.alignRobotToTargetCommand(),
        self.alignLauncherToTargetCommand()
      ).withTimeout(constants.Game.Commands.kScoringAlignmentTimeout),
      self.runLauncherCommand()
    ).withName("GameCommands:ScoreToTarget")

  def runClimberSetupCommand(self) -> Command:
    return cmd.sequence(
      cmd.runOnce(lambda: self.robot.climberDistanceSensor.resetTrigger()),
      cmd.parallel(
        self.robot.launcherArmSubsystem.alignToPositionCommand(constants.Subsystems.Launcher.Arm.kPositionClimber),
        self.robot.climberSubsystem.moveArmUpCommand()
      )
    ).withName("GameCommands:RunClimberSetup")
  
  def runClimberEngageCommand(self) -> Command:
    return cmd.race(
      self.robot.climberSubsystem.moveArmDownCommand(),
      cmd.sequence(
        cmd.waitSeconds(2.5),
        self.robot.climberSubsystem.lockArmCommand().withTimeout(3.0),
        self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
      )
    ).withName("GameCommands:RunClimberEngage")
  
  def rumbleControllersCommand(self, mode: ControllerRumbleMode, pattern: ControllerRumblePattern) -> Command:
    return cmd.parallel(
      self.robot.driverController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Driver or mode == ControllerRumbleMode.Both),
      self.robot.operatorController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Operator or mode == ControllerRumbleMode.Both)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName("GameCommands:RumbleControllers")