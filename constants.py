import math
from wpilib import ADIS16470_IMU, SPI
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import units
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from rev import CANSparkBase
from photonlibpy.photonPoseEstimator import PoseStrategy
from pathplannerlib.controller import PIDConstants as PathPlannerPIDConstants
from pathplannerlib.pathfinding import PathConstraints
from pathplannerlib.path import PathPlannerPath
from lib.classes import PIDConstants
from classes import AutoPath, LauncherRollersSpeeds, LauncherArmPositionTarget

class Power:
  kPowerDistributionCANId: int = 1

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: float = 0.1

class Subsystems:
  class Drive:
    kTrackWidth: float = units.inchesToMeters(24.5)
    kWheelBase: float = units.inchesToMeters(21.5)
    kDriveBaseRadius: float = Translation2d().distance(Translation2d(kWheelBase / 2, kTrackWidth / 2))

    kMaxSpeedMetersPerSecond: float = 6.32
    kMaxAngularSpeed: float = 4 * math.pi

    kDriveInputLimiter: float = 0.6
    kDriveInputRateLimit: float = 0.5

    kDriftCorrectionThetaControllerPIDConstants = PIDConstants(0.01, 0, 0, 0)
    kDriftCorrectionThetaControllerPositionTolerance: float = 0.5
    kDriftCorrectionThetaControllerVelocityTolerance: float = 0.5

    kTargetAlignmentThetaControllerPIDConstants = PIDConstants(0.1, 0, 0.01, 0)
    kTargetAlignmentThetaControllerPositionTolerance: float = 1.0
    kTargetAlignmentThetaControllerVelocityTolerance: float = 1.0

    kPathFollowerTranslationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFollowerRotationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFindingConstraints = PathConstraints(5.8, 3.6, units.degreesToRadians(540), units.degreesToRadians(720))

    kSwerveModuleFrontLeftDrivingMotorCANId: int = 3
    kSwerveModuleFrontLeftTurningMotorCANId: int = 4
    kSwerveModuleFrontRightDrivingMotorCANId: int = 7
    kSwerveModuleFrontRightTurningMotorCANId: int = 8
    kSwerveModuleRearLeftDrivingMotorCANId: int = 5
    kSwerveModuleRearLeftTurningMotorCANId: int = 6
    kSwerveModuleRearRightDrivingMotorCANId: int = 9
    kSwerveModuleRearRightTurningMotorCANId: int = 10

    kSwerveModuleFrontLeftOffset: float = -math.pi / 2
    kSwerveModuleFrontRightOffset: float = 0
    kSwerveModuleRearLeftOffset: float = math.pi
    kSwerveModuleRearRightOffset: float = math.pi / 2

    kSwerveModuleFrontLeftTranslation = Translation2d(kWheelBase / 2, kTrackWidth / 2)
    kSwerveModuleFrontRightTranslation =Translation2d(kWheelBase / 2, -kTrackWidth / 2)
    kSwerveModuleRearLeftTranslation = Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    kSwerveModuleRearRightTranslation = Translation2d(-kWheelBase / 2, -kTrackWidth / 2)

    kSwerveDriveKinematics = SwerveDrive4Kinematics(
      kSwerveModuleFrontLeftTranslation, 
      kSwerveModuleFrontRightTranslation, 
      kSwerveModuleRearLeftTranslation, 
      kSwerveModuleRearRightTranslation
    )

    class SwerveModule:
      kDrivingMotorPinionTeeth: int = 14
      kFreeSpeedRpm: float = 6238.73054766
      kWheelDiameterMeters: float = units.inchesToMeters(3.0)
      kWheelCircumferenceMeters: float = kWheelDiameterMeters * math.pi
      kDrivingMotorReduction: float = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15)
      kDrivingMotorFreeSpeedRps: float = kFreeSpeedRpm / 60
      kDriveWheelFreeSpeedRps: float = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction 
      kDrivingEncoderPositionConversionFactor: float = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
      kDrivingEncoderVelocityConversionFactor: float = ((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) / 60.0
      kTurningEncoderInverted: bool = True
      kTurningEncoderPositionConversionFactor: float = 2 * math.pi
      kTurningEncoderVelocityConversionFactor: float = (2 * math.pi) / 60.0
      kTurningEncoderPositionPIDMinInput: float = 0
      kTurningEncoderPositionPIDMaxInput: float = kTurningEncoderPositionConversionFactor
      kDrivingMotorCurrentLimit: int = 80
      kDrivingMotorMaxReverseOutput: float = -1.0
      kDrivingMotorMaxForwardOutput: float = 1.0
      kDrivingMotorIdleMode = CANSparkBase.IdleMode.kBrake
      kDrivingMotorPIDConstants = PIDConstants(0.04, 0, 0, 1 / kDriveWheelFreeSpeedRps)
      kTurningMotorCurrentLimit: int = 20
      kTurningMotorMaxReverseOutput: float = -1.0
      kTurningMotorMaxForwardOutput: float = 1.0
      kTurningMotorIdleMode = CANSparkBase.IdleMode.kBrake
      kTurningMotorPIDConstants = PIDConstants(1, 0, 0, 0)

  class Intake:
    kTopBeltsMotorCANId: int = 18
    kBottomBeltsMotorCANId: int = 19
    kRollersMotorCANId: int = 20

    kTopBeltsMotorCurrentLimit: int = 60
    kTopBeltsMotorMaxForwardOutput: float = 0.6
    kTopBeltsMotorMaxReverseOutput: float = -0.6
    kTopBeltsMotorIdleMode = CANSparkBase.IdleMode.kBrake

    kBottomBeltsMotorCurrentLimit: int = 60
    kBottomBeltsMotorMaxForwardOutput: float = 0.6
    kBottomBeltsMotorMaxReverseOutput: float = -0.6
    kBottomBeltsMotorIdleMode = CANSparkBase.IdleMode.kCoast

    kRollersMotorCurrentLimit: int = 60
    kRollersMotorMaxForwardOutput: float = 0.6
    kRollersMotorMaxReverseOutput: float = -0.6
    kRollersMotorIdleMode = CANSparkBase.IdleMode.kBrake

    kIntakeBeltsSpeed: float = 0.8
    kIntakeBeltsAutoSpeed: float = 0.74
    kIntakeBeltsAdjustPositionSpeed: float = 0.3

    kIntakeCompletionDelay: units.seconds = 0.033
    kIntakeCompletionAutoDelay: units.seconds = 0.019
    kIntakeAdjustmentDelay: units.seconds = 0.05
    kIntakeReloadDelay: units.seconds = 0.25

  class Launcher:
    kArmMotorCANId: int = 11
    kTopRollerMotorCANId: int = 12
    kBottomRollerMotorCANId: int = 13

    kArmMotorCurrentLimit: int = 60
    kArmMotorMaxReverseOutput: float = -1.0
    kArmMotorMaxForwardOutput: float = 1.0
    kArmMotorIdleMode = CANSparkBase.IdleMode.kBrake
    kArmMotorPIDConstants = PIDConstants(0.0003, 0, 0.00015, 1 / 16.8)
    kArmMotorForwardSoftLimit: float = 14.5
    kArmMotorReverseSoftLimit: float = 1
    kArmMotorPositionConversionFactor: float = 1.0 / 3.0
    kArmMotorVelocityConversionFactor: float = kArmMotorPositionConversionFactor / 60.0
    kArmMotorSmartMotionMaxVelocity: float = (33.0 / kArmMotorPositionConversionFactor) * 60
    kArmMotorSmartMotionMaxAccel: float = 100.0 / kArmMotorVelocityConversionFactor

    kTopRollerMotorCurrentLimit = 100
    kTopRollerMotorMaxForwardOutput: float = 1.0
    kTopRollerMotorMaxReverseOutput: float = -1.0
    kTopRollerMotorIdleMode = CANSparkBase.IdleMode.kBrake

    kBottomRollerMotorCurrentLimit = 100
    kBottomRollerMotorMaxForwardOutput: float = 1.0
    kBottomRollerMotorMaxReverseOutput: float = -1.0
    kBottomRollerMotorIdleMode = CANSparkBase.IdleMode.kBrake

    kRollersLaunchStartDelay: units.seconds = 0.5

    kRollersSpeedsSpeaker = LauncherRollersSpeeds(0.8, 0.8)
    kRollersSpeedsAmp = LauncherRollersSpeeds(0.27, 0.27)
    kRollersSpeedsShuttle = LauncherRollersSpeeds(0.6, 0.6)
    kRollersSpeedsWarmup = LauncherRollersSpeeds(0.6, 0.6)

    kArmInputLimiter: float = 0.5
    kArmTargetAlignmentPositionTolerance: float = 0.1
    
    kArmPositionIntake: float = 7.0
    kArmPositionAmp: float = 13
    kArmPositionShuttle: float = 12.0
    kArmPositionSubwoofer: float = 12.9
    kArmPositionPodium: float = 10.35
    kArmPositionFlat: float = 1.0

    kArmPositionTargets: list[LauncherArmPositionTarget] = [
      LauncherArmPositionTarget(1.00, 13.0),
      LauncherArmPositionTarget(1.35, 12.6),
      LauncherArmPositionTarget(2.3, 8.15),
      LauncherArmPositionTarget(3.65, 5.8),
      LauncherArmPositionTarget(5.0, 4.17),
      LauncherArmPositionTarget(6.2, 3.94)
    ]

  class Climber:
    kArmLeftMotorCANId: int = 16
    kArmRightMotorCANId: int = 17

    kArmMotorCurrentLimit: int = 100
    kArmMotorMaxReverseOutput: float = -1.0
    kArmMotorMaxForwardOutput: float = 1.0
    kArmMotorIdleMode = CANSparkBase.IdleMode.kBrake
    kArmMotorPIDConstants = PIDConstants(0.05, 0, 0, 0)
    kArmMotorForwardSoftLimit: float = 33.0
    kArmMotorReverseSoftLimit: float = 0.0

    kArmPositionDefault = 8.2

    kBrakeServoChannel: int = 9

class Sensors:
  class Gyro:
    kIMUAxisYaw = ADIS16470_IMU.IMUAxis.kZ
    kIMUAxisRoll = ADIS16470_IMU.IMUAxis.kY
    kIMUAxisPitch = ADIS16470_IMU.IMUAxis.kX
    kSPIPort = SPI.Port.kOnboardCS0
    kInitCalibrationTime = ADIS16470_IMU.CalibrationTime._8s
    kCommandCalibrationTime = ADIS16470_IMU.CalibrationTime._4s
    kCommandCalibrationDelay: units.seconds = 4.0

  class Pose:
    kPoseSensors: dict[str, Transform3d] = {
      "Rear": Transform3d(
        Translation3d(units.inchesToMeters(-3.25), units.inchesToMeters(-10.75), units.inchesToMeters(18.0)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-24.0), units.degreesToRadians(180.0))
      ),
      "Side": Transform3d(
        Translation3d(units.inchesToMeters(-3.25), units.inchesToMeters(-11.5), units.inchesToMeters(15.5)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-28.4), units.degreesToRadians(-90.0))
      )
    }
    kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kVisionSingleTagStandardDeviations: tuple[float, ...] = [1.0, 1.0, 2.0]
    kVisionMultiTagStandardDeviations: tuple[float, ...] = [0.5, 0.5, 1.0]
    kVisionMaxPoseAmbiguity: float = 0.2

  class BreamBreak:
    class LauncherBottom:
      kSensorName = "LauncherBottom"
      kChannel: int = 7
    class LauncherTop:
      kSensorName = "LauncheTop"
      kChannel: int = 5
    class Climber:
      kSensorName = "Climber"
      kChannel: int = 3

  class Object:
    kCameraName = "Front"
    kObjectName = "Note"

_aprilTagFieldLayout = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)

class Game:
  class Field:
    kAprilTagFieldLayout = _aprilTagFieldLayout

    class Targets:  
      kBlueSpeaker = _aprilTagFieldLayout.getTagPose(7) or Pose3d()
      kRedSpeaker = _aprilTagFieldLayout.getTagPose(4) or Pose3d()
      kBlueAmp = _aprilTagFieldLayout.getTagPose(5) or Pose3d()
      kRedAmp = _aprilTagFieldLayout.getTagPose(6) or Pose3d()

      kSpeakerTargetYawTransformX: float = units.inchesToMeters(6.0)
      kSpeakerTargetYTransform: float = units.inchesToMeters(-6.0)
      kSpeakerTargetPitchTransformZ: float = units.inchesToMeters(24)
      kSpeakerTargetDistanceTransformX: float = units.inchesToMeters(0)

  class Auto:
    kPaths: dict[AutoPath, PathPlannerPath] = {
      AutoPath.ScorePreload1: PathPlannerPath.fromPathFile(AutoPath.ScorePreload1.name),
      AutoPath.ScorePreload2: PathPlannerPath.fromPathFile(AutoPath.ScorePreload2.name),
      AutoPath.ScorePreload3: PathPlannerPath.fromPathFile(AutoPath.ScorePreload3.name),
      AutoPath.Pickup1: PathPlannerPath.fromPathFile(AutoPath.Pickup1.name),
      AutoPath.Pickup13: PathPlannerPath.fromPathFile(AutoPath.Pickup13.name),
      AutoPath.Pickup2: PathPlannerPath.fromPathFile(AutoPath.Pickup2.name),
      AutoPath.Pickup21: PathPlannerPath.fromPathFile(AutoPath.Pickup21.name),
      AutoPath.Pickup23: PathPlannerPath.fromPathFile(AutoPath.Pickup23.name),
      AutoPath.Pickup3: PathPlannerPath.fromPathFile(AutoPath.Pickup3.name),
      AutoPath.Pickup31: PathPlannerPath.fromPathFile(AutoPath.Pickup31.name),
      AutoPath.Pickup4: PathPlannerPath.fromPathFile(AutoPath.Pickup4.name),
      AutoPath.Pickup5: PathPlannerPath.fromPathFile(AutoPath.Pickup5.name),
      AutoPath.Pickup61: PathPlannerPath.fromPathFile(AutoPath.Pickup61.name),
      AutoPath.Pickup62: PathPlannerPath.fromPathFile(AutoPath.Pickup62.name),
      AutoPath.Pickup63: PathPlannerPath.fromPathFile(AutoPath.Pickup63.name),
      AutoPath.Pickup72: PathPlannerPath.fromPathFile(AutoPath.Pickup72.name),
      AutoPath.Pickup73: PathPlannerPath.fromPathFile(AutoPath.Pickup73.name),
      AutoPath.Pickup8: PathPlannerPath.fromPathFile(AutoPath.Pickup8.name),
      AutoPath.ScoreStage1: PathPlannerPath.fromPathFile(AutoPath.ScoreStage1.name),
      AutoPath.ScoreStage2: PathPlannerPath.fromPathFile(AutoPath.ScoreStage2.name),
      AutoPath.ScoreStage3: PathPlannerPath.fromPathFile(AutoPath.ScoreStage3.name)
    }

    kPickupTimeout: units.seconds = 4.5

