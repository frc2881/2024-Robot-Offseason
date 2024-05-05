from wpilib import SmartDashboard, SendableChooser, PowerDistribution, DriverStation
from commands2 import Command, cmd
from commands2.button import Trigger
from pathplannerlib.auto import AutoBuilder, HolonomicPathFollowerConfig, ReplanningConfig
from lib import utils
from lib.classes import Alliance, RobotState
from lib.controllers.game_controller import GameController
from lib.controllers.lights_controller import LightsController
from lib.sensors.gyro_sensor import GyroSensor
from lib.sensors.pose_sensor import PoseSensor
from lib.sensors.beambreak_sensor import BeamBreakSensor
from lib.sensors.object_sensor import ObjectSensor
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.localization_subsystem import LocalizationSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.launcher_arm_subsystem import LauncherArmSubsystem
from subsystems.launcher_rollers_subsystem import LauncherRollersSubsystem
from subsystems.climber_subsystem import ClimberSubsystem
from commands.game_commands import GameCommands
from commands.auto_commands import AutoCommands
from classes import LightsMode
import constants

class RobotContainer:
  def __init__(self) -> None:
    self._initPower()
    self._initSensors()
    self._initSubsystems()
    self._initControllers()
    self._initCommands()
    self._setupControllers()
    self._setupTriggers()
    self._setupAutos()

  def _initPower(self) -> None:
    self.powerDistribution = PowerDistribution(
      constants.Power.kPowerDistributionCANId, 
      PowerDistribution.ModuleType.kRev
    )

  def _initSensors(self) -> None:
    self.gyroSensor = GyroSensor(
      constants.Sensors.Gyro.kIMUAxisYaw,
      constants.Sensors.Gyro.kIMUAxisPitch,
      constants.Sensors.Gyro.kIMUAxisRoll,
      constants.Sensors.Gyro.kSPIPort,
      constants.Sensors.Gyro.kInitCalibrationTime,
      constants.Sensors.Gyro.kCommandCalibrationTime,
      constants.Sensors.Gyro.kCommandCalibrationDelay
    )
    self.poseSensors: list[PoseSensor] = []
    for cameraName, cameraTransform in constants.Sensors.Pose.kPoseSensors.items():
      self.poseSensors.append(PoseSensor(
        cameraName,
        cameraTransform,
        constants.Sensors.Pose.kPoseStrategy,
        constants.Sensors.Pose.kFallbackPoseStrategy,
        constants.Game.Field.kAprilTagFieldLayout
      ))
    self.launcherBottomBeamBreakSensor = BeamBreakSensor(
      constants.Sensors.BeamBreak.LauncherBottom.kSensorName,
      constants.Sensors.BeamBreak.LauncherBottom.kChannel
    )
    self.launcherTopBeamBreakSensor = BeamBreakSensor(
      constants.Sensors.BeamBreak.LauncherTop.kSensorName,
      constants.Sensors.BeamBreak.LauncherTop.kChannel
    )
    self.climberBeamBreakSensor = BeamBreakSensor(
      constants.Sensors.BeamBreak.Climber.kSensorName,
      constants.Sensors.BeamBreak.Climber.kChannel
    )
    self.objectSensor = ObjectSensor(
      constants.Sensors.Object.kCameraName,
      constants.Sensors.Object.kObjectName
    )

  def _initSubsystems(self) -> None:
    self.driveSubsystem = DriveSubsystem(
      lambda: self.gyroSensor.getHeading()
    )
    self.localizationSubsystem = LocalizationSubsystem(
      self.poseSensors,
      lambda: self.gyroSensor.getRotation(),
      lambda: self.driveSubsystem.getSwerveModulePositions()
    )
    self.intakeSubsystem = IntakeSubsystem()
    self.launcherArmSubsystem = LauncherArmSubsystem()
    self.launcherRollersSubsystem = LauncherRollersSubsystem()
    self.climberSubsystem = ClimberSubsystem()
    
  def _initControllers(self) -> None:
    DriverStation.silenceJoystickConnectionWarning(True)
    self.driverController = GameController(
      constants.Controllers.kDriverControllerPort, 
      constants.Controllers.kInputDeadband
    )
    self.operatorController = GameController(
      constants.Controllers.kOperatorControllerPort, 
      constants.Controllers.kInputDeadband
    )
    self.lightsController = LightsController()

  def _initCommands(self) -> None:
    self.gameCommands = GameCommands(self)
    self.autoCommands = AutoCommands(self.gameCommands)

  def _setupControllers(self) -> None:
    # DRIVER ========================================
    self.driveSubsystem.setDefaultCommand(
      self.driveSubsystem.driveWithControllerCommand(
        lambda: self.driverController.getLeftY(),
        lambda: self.driverController.getLeftX(),
        lambda: self.driverController.getRightX()
      )
    )
    self.driverController.rightTrigger().whileTrue(self.gameCommands.runIntakeCommand())
    self.driverController.rightBumper().whileTrue(self.gameCommands.ejectIntakeCommand())
    self.driverController.leftTrigger().whileTrue(self.gameCommands.alignLauncherToPositionCommand(constants.Subsystems.Launcher.kArmPositionShuttle))
    self.driverController.leftBumper().whileTrue(self.gameCommands.runClimberSetupCommand()).onFalse (self.gameCommands.runClimberEngageCommand())
    self.driverController.rightStick().whileTrue(self.gameCommands.alignRobotToTargetCommand())
    self.driverController.leftStick().whileTrue(self.driveSubsystem.setLockedCommand())
    self.driverController.povUp().whileTrue(self.climberSubsystem.moveArmUpCommand())
    self.driverController.povDown().whileTrue(self.climberSubsystem.moveArmDownCommand())
    self.driverController.povLeft().whileTrue(self.climberSubsystem.moveArmToDefaultPositionCommand())
    self.driverController.povRight().whileTrue(self.climberSubsystem.unlockArmCommand())
    self.driverController.a().whileTrue(self.gameCommands.alignRobotToTargetCommand())
    # self.driverController.b().whileTrue(cmd.none())
    self.driverController.y().whileTrue(self.gameCommands.reloadIntakeCommand())
    self.driverController.x().whileTrue(self.gameCommands.runLauncherCommand(constants.Subsystems.Launcher.kRollersSpeedsShuttle))
    self.driverController.start().onTrue(self.gyroSensor.calibrateCommand())
    self.driverController.back().onTrue(self.gyroSensor.resetCommand())

    # OPERATOR ========================================
    self.launcherArmSubsystem.setDefaultCommand(
      self.launcherArmSubsystem.runCommand(
        lambda: self.operatorController.getLeftY()
      )
    )
    self.operatorController.rightTrigger().whileTrue(self.gameCommands.runLauncherCommand(constants.Subsystems.Launcher.kRollersSpeedsSpeaker))
    self.operatorController.rightBumper().whileTrue(self.gameCommands.runLauncherCommand(constants.Subsystems.Launcher.kRollersSpeedsAmp))
    self.operatorController.leftTrigger().whileTrue(self.gameCommands.alignLauncherToTargetCommand())
    self.operatorController.leftBumper().whileTrue(self.gameCommands.alignLauncherToPositionCommand(constants.Subsystems.Launcher.kArmPositionAmp))
    # self.operatorController.rightStick().whileTrue(cmd.none())
    # self.operatorController.leftStick().whileTrue(cmd.none())
    self.operatorController.povUp().whileTrue(self.gameCommands.alignLauncherToPositionCommand(constants.Subsystems.Launcher.kArmPositionPodium))
    self.operatorController.povDown().whileTrue(self.gameCommands.alignLauncherToPositionCommand(constants.Subsystems.Launcher.kArmPositionSubwoofer))
    # self.operatorController.povLeft().whileTrue(cmd.none())
    # self.operatorController.povRight().whileTrue(cmd.none())
    # self.operatorController.a().whileTrue(cmd.none())
    # self.operatorController.b().whileTrue(cmd.none())
    # self.operatorController.y().whileTrue(cmd.none())
    # self.operatorController.x().whileTrue(cmd.none())
    self.operatorController.start().whileTrue(self.climberSubsystem.resetToZeroCommand())
    self.operatorController.back().whileTrue(self.launcherArmSubsystem.resetToZeroCommand())

  def _setupTriggers(self) -> None:
    self._setLightsModeTrigger()
    self._setLockClimberArmTrigger()

  def _setLightsModeTrigger(self) -> None:
    Trigger(
      lambda: self.launcherBottomBeamBreakSensor.hasTarget()
    ).whileTrue(
      cmd.run(lambda: self._setLightsMode()).ignoringDisable(True)
    ).onFalse(
      cmd.runOnce(lambda: self.lightsController.setLightsMode(LightsMode.Default)).ignoringDisable(True)
    )

  def _setLightsMode(self) -> None:
    lightsMode: LightsMode = LightsMode.IntakeReady
    if self.launcherTopBeamBreakSensor.hasTarget():
      lightsMode = LightsMode.IntakeNotReady
    else:
      if utils.getRobotState() == RobotState.Disabled:
        if self.localizationSubsystem.hasVisionTargets():
          lightsMode = LightsMode.VisionNotReady
      else:
        if self.driveSubsystem.isAlignedToTarget() and self.launcherArmSubsystem.isAlignedToTarget():
          lightsMode = LightsMode.LaunchReady
    self.lightsController.setLightsMode(lightsMode)

  def _setLockClimberArmTrigger(self) -> None:
    Trigger(
      lambda: utils.isValueInRange(utils.getMatchTime(), 0.1, 1.0)
    ).onTrue(
      lambda: self.climberSubsystem.lockArmCommand().withTimeout(3.0)
    )

  def _setupAutos(self) -> None:
    AutoBuilder.configureHolonomic(
      lambda: self.localizationSubsystem.getPose(), 
      lambda: self.localizationSubsystem.resetPose(), 
      lambda: self.driveSubsystem.getSpeeds(), 
      lambda: self.driveSubsystem.drive(), 
      HolonomicPathFollowerConfig(
        constants.Subsystems.Drive.kPathFollowerTranslationPIDConstants,
        constants.Subsystems.Drive.kPathFollowerRotationPIDConstants,
        constants.Subsystems.Drive.kMaxSpeedMetersPerSecond, 
        constants.Subsystems.Drive.kDriveBaseRadius, 
        ReplanningConfig(True, True)
      ),
      lambda: utils.getAlliance() == Alliance.Red,
      self.driveSubsystem
    )

    self._autoChooser = SendableChooser()
    self._autoChooser.setDefaultOption("None", lambda: cmd.none())

    self._autoChooser.addOption("[ 1 ] 0", lambda: self.autoCommands.auto0())
    self._autoChooser.addOption("[ 1 ] 0_1", lambda: self.autoCommands.auto10_1())
    self._autoChooser.addOption("[ 1 ] _0_1", lambda: self.autoCommands.auto1_0_1())
    self._autoChooser.addOption("[ 1 ] _0_1_2_3", lambda: self.autoCommands.auto1_0_1_2_3())
    self._autoChooser.addOption("[ 1 ] _0_1_2_3_83", lambda: self.autoCommands.auto1_0_1_2_3_83())
    self._autoChooser.addOption("[ 1 ] _0_1_2_62", lambda: self.autoCommands.auto1_0_1_2_62())
    self._autoChooser.addOption("[ 1 ] _0_1_41", lambda: self.autoCommands.auto1_0_1_41())
    self._autoChooser.addOption("[ 1 ] _0_1_41_51", lambda: self.autoCommands.auto1_0_1_41_51())
    self._autoChooser.addOption("[ 1 ] _0_1_51", lambda: self.autoCommands.auto1_0_1_51())
    self._autoChooser.addOption("[ 1 ] _0_1_51_41", lambda: self.autoCommands.auto1_0_1_51_41())
    self._autoChooser.addOption("[ 1 ] _0_1_51_61", lambda: self.autoCommands.auto1_0_1_51_61())
    self._autoChooser.addOption("[ 1 ] _0_1_51_62", lambda: self.autoCommands.auto1_0_1_51_62())

    self._autoChooser.addOption("[ 2 ] 0", lambda: self.autoCommands.auto0())
    self._autoChooser.addOption("[ 2 ] 0_2", lambda: self.autoCommands.auto20_2())
    self._autoChooser.addOption("[ 2 ] _0_2", lambda: self.autoCommands.auto2_0_2()) 
    self._autoChooser.addOption("[ 2 ] _0_2_1", lambda: self.autoCommands.auto2_0_2_1()) 
    self._autoChooser.addOption("[ 2 ] _0_2_1_3", lambda: self.autoCommands.auto2_0_2_1_3()) 
    self._autoChooser.addOption("[ 2 ] _0_2_3", lambda: self.autoCommands.auto2_0_2_3()) 
    self._autoChooser.addOption("[ 2 ] _0_2_3_1", lambda: self.autoCommands.auto2_0_2_3_1()) 
    self._autoChooser.addOption("[ 2 ] _0_2_62", lambda: self.autoCommands.auto2_0_2_62()) 
    self._autoChooser.addOption("[ 2 ] _0_2_62_51", lambda: self.autoCommands.auto2_0_2_62_51()) 
    self._autoChooser.addOption("[ 2 ] _0_2_62_72", lambda: self.autoCommands.auto2_0_2_62_72())
    self._autoChooser.addOption("[ 2 ] _0_2_72", lambda: self.autoCommands.auto2_0_2_72())
    self._autoChooser.addOption("[ 2 ] _0_2_72_62", lambda: self.autoCommands.auto2_0_2_72_62()) 

    self._autoChooser.addOption("[ 3 ] 0", lambda: self.autoCommands.auto0())
    self._autoChooser.addOption("[ 3 ] 0_3", lambda: self.autoCommands.auto30_3())
    self._autoChooser.addOption("[ 3 ] 0_73", lambda: self.autoCommands.auto30_73())
    self._autoChooser.addOption("[ 3 ] 0_83", lambda: self.autoCommands.auto30_83())
    self._autoChooser.addOption("[ 3 ] 0_73_83", lambda: self.autoCommands.auto30_73_83())
    self._autoChooser.addOption("[ 3 ] 0_83_73", lambda: self.autoCommands.auto30_83_73())
    self._autoChooser.addOption("[ 3 ] _0_3", lambda: self.autoCommands.auto3_0_3())
    self._autoChooser.addOption("[ 3 ] _0_3_2_1", lambda: self.autoCommands.auto3_0_3_2_1())
    self._autoChooser.addOption("[ 3 ] _0_3_2_1_41", lambda: self.autoCommands.auto3_0_3_2_1_41()) 
    self._autoChooser.addOption("[ 3 ] _0_3_2_1_51", lambda: self.autoCommands.auto3_0_3_2_1_51()) 
    self._autoChooser.addOption("[ 3 ] _0_3_2_62", lambda: self.autoCommands.auto3_0_3_2_62()) 
    self._autoChooser.addOption("[ 3 ] _0_3_62", lambda: self.autoCommands.auto3_0_3_62()) 
    self._autoChooser.addOption("[ 3 ] _0_3_62_72", lambda: self.autoCommands.auto3_0_3_62_72()) 
    self._autoChooser.addOption("[ 3 ] _0_3_72", lambda: self.autoCommands.auto3_0_3_72()) 
    self._autoChooser.addOption("[ 3 ] _0_3_72_62", lambda: self.autoCommands.auto3_0_3_72_62()) 
    self._autoChooser.addOption("[ 3 ] _0_3_73", lambda: self.autoCommands.auto3_0_3_73())
    self._autoChooser.addOption("[ 3 ] _0_3_73_83", lambda: self.autoCommands.auto3_0_3_73_83())
    self._autoChooser.addOption("[ 3 ] _0_3_82", lambda: self.autoCommands.auto3_0_3_82()) 
    self._autoChooser.addOption("[ 3 ] _0_3_83", lambda: self.autoCommands.auto3_0_3_83()) 
    self._autoChooser.addOption("[ 3 ] _0_3_82_62", lambda: self.autoCommands.auto3_0_3_82_62()) 
    self._autoChooser.addOption("[ 3 ] _0_3_82_72", lambda: self.autoCommands.auto3_0_3_82_72()) 
    self._autoChooser.addOption("[ 3 ] _0_3_83_62", lambda: self.autoCommands.auto3_0_3_83_62()) 
    self._autoChooser.addOption("[ 3 ] _0_3_83_72", lambda: self.autoCommands.auto3_0_3_83_72()) 
    self._autoChooser.addOption("[ 3 ] _0_3_83_73", lambda: self.autoCommands.auto3_0_3_83_73()) 

    SmartDashboard.putData("Robot/Auto/Command", self._autoChooser)
    
  def getAutonomousCommand(self) -> Command:
    return self._autoChooser.getSelected()()
    
  def autonomousInit(self) -> None:
    self._resetRobot()

  def teleopInit(self) -> None:
    self._resetRobot()
    self.gyroSensor.setRobotToField(self.localizationSubsystem.getPose())

  def testInit(self) -> None:
    self._resetRobot()

  def _resetRobot(self) -> None:
    self.driveSubsystem.reset()
    self.intakeSubsystem.reset()
    self.launcherArmSubsystem.reset()
    self.launcherRollersSubsystem.reset()
    self.climberSubsystem.reset()

  def updateTelemetry(self) -> None:
    self.gyroSensor.updateTelemetry()
    for poseSensor in self.poseSensors:
      poseSensor.updateTelemetry()
    self.launcherBottomBeamBreakSensor.updateTelemetry()
    self.launcherTopBeamBreakSensor.updateTelemetry()
    self.climberBeamBreakSensor.updateTelemetry()
    self.objectSensor.updateTelemetry()

    SmartDashboard.putNumber("Robot/Power/TotalCurrent", self.powerDistribution.getTotalCurrent())

    SmartDashboard.putBoolean(
      "Robot/HasInitialZeroResets", 
      (self.launcherArmSubsystem.hasInitialZeroReset() and self.climberSubsystem.hasInitialZeroReset()) or utils.isCompetitionMode()
    )
