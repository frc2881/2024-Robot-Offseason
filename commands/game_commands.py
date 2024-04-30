from typing import TYPE_CHECKING
from commands2 import Command, cmd
if TYPE_CHECKING: from robot_container import RobotContainer
from lib import utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
from classes import LauncherRollersSpeeds
import constants

class GameCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self.robot = robot

  def runIntakeCommand(self) -> Command:
    return cmd.sequence(
      self.robot.intakeSubsystem.runCommand(
        lambda: self.robot.launcherBottomBeamBreakSensor.hasTarget()
      ).deadlineWith(
        self.robot.launcherArmSubsystem.alignToPositionCommand(constants.Subsystems.Launcher.kArmPositionIntake)
      ),
      cmd.waitSeconds(constants.Subsystems.Intake.kIntakeAdjustmentDelay),
      self.robot.intakeSubsystem.adjustPositionCommand(lambda: self.robot.launcherTopBeamBreakSensor.hasTarget()),
      cmd.either(self.rumbleControllersCommand(ControllerRumbleMode.Driver), cmd.none(), lambda: not utils.isAutonomousMode())
    ).withName("RunIntake")
  
  def ejectIntakeCommand(self) -> Command:
    return self.robot.intakeSubsystem.ejectCommand().withName("EjectIntake")
  
  def reloadIntakeCommand(self) -> Command:
    return cmd.sequence(
      self.robot.intakeSubsystem.ejectCommand(),
      cmd.waitSeconds(constants.Subsystems.Intake.kIntakeReloadDelay),
      self.runIntakeCommand()
    ).withName("ReloadIntake")

  def alignRobotToTargetCommand(self) -> Command:
    return cmd.sequence(
      cmd.either(self.rumbleControllersCommand(ControllerRumbleMode.Operator), cmd.none(), lambda: not utils.isAutonomousMode()),
      self.robot.driveSubsystem.alignToTargetCommand(
        lambda: self.robot.localizationSubsystem.getPose(), 
        lambda: self.robot.localizationSubsystem.getTargetYaw()
      ).withTimeout(utils.getValueForRobotMode(2.0, float("inf"))),
      cmd.either(self.rumbleControllersCommand(ControllerRumbleMode.Driver), cmd.none(), lambda: not utils.isAutonomousMode())
    ).withName("AlignRobotToTarget")

  def alignLauncherToTargetCommand(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self.robot.launcherArmSubsystem.alignToTargetCommand(lambda: self.robot.localizationSubsystem.getTargetDistance()),
        self.robot.launcherRollersSubsystem.runCommand(constants.Subsystems.Launcher.kRollersSpeedsWarmup)
      ).withTimeout(utils.getValueForRobotMode(2.0, float("inf")))
    ).withName("AlignLauncherToTarget")

  def alignLauncherToPositionCommand(self, position: float) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self.robot.launcherArmSubsystem.alignToPositionCommand(position),
        self.robot.launcherRollersSubsystem.runCommand(constants.Subsystems.Launcher.kRollersSpeedsWarmup)
      ).withTimeout(utils.getValueForRobotMode(2.0, float("inf")))
    ).withName("AlignLauncherToPosition")
  
  def runLauncherCommand(self, launcherRollerSpeeds: LauncherRollersSpeeds) -> Command:
    return cmd.parallel(
      self.robot.launcherRollersSubsystem.runCommand(launcherRollerSpeeds),
      cmd.sequence(
        cmd.waitSeconds(constants.Subsystems.Launcher.kRollersLaunchStartDelay),
        self.robot.intakeSubsystem.launchCommand()
      )
    ).onlyIf(
      lambda: self.robot.launcherBottomBeamBreakSensor.hasTarget()
    ).until(
      lambda: not self.robot.launcherBottomBeamBreakSensor.hasTarget() and not self.robot.launcherTopBeamBreakSensor.hasTarget()
    ).finallyDo(lambda end: [
      self.robot.driveSubsystem.clearTargetAlignment(),
      self.robot.launcherArmSubsystem.clearTargetAlignment()
    ]).withName("RunLauncher")

  def runClimberSetupCommand(self) -> Command:
    return cmd.sequence(
      cmd.runOnce(lambda: self.robot.climberBeamBreakSensor.resetTrigger()),
      cmd.parallel(
        self.robot.launcherArmSubsystem.alignToPositionCommand(constants.Subsystems.Launcher.kArmPositionFlat),
        self.robot.climberSubsystem.moveArmUpCommand()
      )
    ).withName("RunClimberSetupCommand")
  
  def runClimberEngageCommand(self) -> Command:
    return cmd.race(
      self.robot.climberSubsystem.moveArmDownCommand(),
      cmd.sequence(
        cmd.waitSeconds(2.5),
        self.robot.climberSubsystem.lockArmCommand().withTimeout(3.0),
        self.rumbleControllersCommand(ControllerRumbleMode.Driver).withTimeout(1.0)
      )
    ).withName("RunClimberEngageCommand")
  
  def rumbleControllersCommand(self, mode: ControllerRumbleMode) -> Command:
    return cmd.parallel(
      self.robot.driverController.rumbleCommand(ControllerRumblePattern.Short)
      .onlyIf(lambda: mode == ControllerRumbleMode.Driver or mode == ControllerRumbleMode.Both),
      self.robot.operatorController.rumbleCommand(ControllerRumblePattern.Short)
      .onlyIf(lambda: mode == ControllerRumbleMode.Operator or mode == ControllerRumbleMode.Both)
    )\
    .withName("RumbleControllers")