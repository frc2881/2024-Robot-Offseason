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
        self.robot.launcherArmSubsystem.alignToPositionCommand(
          constants.Subsystems.Launcher.kArmPositionIntake
        )
      ),
      self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
    ).withName("GameCommands:RunIntake")
  
  def ejectIntakeCommand(self) -> Command:
    return self.robot.intakeSubsystem.ejectCommand().withName("GameCommands:EjectIntake")
  
  # TODO: validate that reload is still needed and if it works correctly with the delay constant
  def reloadIntakeCommand(self) -> Command:
    return cmd.sequence(
      self.robot.intakeSubsystem.ejectCommand(),
      cmd.waitSeconds(constants.Subsystems.Intake.kReloadDelay),
      self.runIntakeCommand(IntakeDirection.Front)
    ).withName("GameCommands:ReloadIntake")

  def alignRobotToTargetCommand(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self.robot.driveSubsystem.alignToTargetCommand(
          lambda: self.robot.localizationSubsystem.getPose(), 
          lambda: self.robot.localizationSubsystem.getTargetYaw()
        ).withTimeout(utils.getValueForRobotMode(2.0, float("inf"))),
        self.rumbleControllersCommand(ControllerRumbleMode.Operator, ControllerRumblePattern.Short)
      ),
      self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
    ).withName("GameCommands:AlignRobotToTarget")

  def alignLauncherToTargetCommand(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self.robot.launcherArmSubsystem.alignToTargetCommand(lambda: self.robot.localizationSubsystem.getTargetDistance()),
        self.runLauncherWarmupCommand()
      ).withTimeout(utils.getValueForRobotMode(2.0, float("inf")))
    ).withName("GameCommands:AlignLauncherToTarget")

  def alignLauncherToPositionCommand(self, position: float) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self.robot.launcherArmSubsystem.alignToPositionCommand(position),
        self.runLauncherWarmupCommand()
      ).withTimeout(utils.getValueForRobotMode(2.0, float("inf")))
    ).withName("GameCommands:AlignLauncherToPosition")
  
  def runLauncherWarmupCommand(self) -> Command:
    return self.robot.launcherRollersSubsystem.runCommand(
      constants.Subsystems.Launcher.kRollersSpeedsWarmup
    ).withName("GameCommands:RunLauncherWarmup")

  def runLauncherCommand(self, launcherRollerSpeeds: LauncherRollersSpeeds) -> Command:
    return cmd.parallel(
      self.robot.launcherRollersSubsystem.runCommand(launcherRollerSpeeds),
      cmd.sequence(
        cmd.waitSeconds(constants.Subsystems.Launcher.kRollersLaunchStartDelay),
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

  def runClimberSetupCommand(self) -> Command:
    return cmd.sequence(
      cmd.runOnce(lambda: self.robot.climberDistanceSensor.resetTrigger()),
      cmd.parallel(
        self.robot.launcherArmSubsystem.alignToPositionCommand(constants.Subsystems.Launcher.kArmPositionClimber),
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