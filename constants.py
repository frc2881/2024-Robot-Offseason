import math
from wpilib import ADIS16470_IMU, SPI
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import units
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from rev import CANSparkBase
from photonlibpy.photonPoseEstimator import PoseStrategy
from extras.pathplannerlib.controller import PIDConstants as PathPlannerPIDConstants
from extras.pathplannerlib.pathfinding import PathConstraints
from extras.pathplannerlib.path import PathPlannerPath
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

    kTargetAlignmentThetaControllerPIDConstants = PIDConstants(0.075, 0, 0, 0)
    kTargetAlignmentThetaControllerPositionTolerance: float = 1.0
    kTargetAlignmentThetaControllerVelocityTolerance: float = 1.0

    kPathFollowerTranslationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFollowerRotationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFindingConstraints = PathConstraints(4.2, 2.4, units.degreesToRadians(540), units.degreesToRadians(720))

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
    kBottomBeltsMotorCANId: int = 18
    kTopRearBeltsMotorCANId: int = 19
    kTopFrontBeltsMotorCANId: int = 20

    kBottomBeltsMotorCurrentLimit: int = 60
    kBottomBeltsMotorMaxForwardOutput: float = 0.8
    kBottomBeltsMotorMaxReverseOutput: float = -0.8
    kBottomBeltsMotorIdleMode = CANSparkBase.IdleMode.kCoast

    kTopFrontBeltsMotorCurrentLimit: int = 60
    kTopFrontBeltsMotorMaxForwardOutput: float = 0.8
    kTopFrontBeltsMotorMaxReverseOutput: float = -0.8
    kTopFrontBeltsMotorIdleMode = CANSparkBase.IdleMode.kBrake

    kTopRearBeltsMotorCurrentLimit: int = 60
    kTopRearBeltsMotorMaxForwardOutput: float = 0.8
    kTopRearBeltsMotorMaxReverseOutput: float = -0.8
    kTopRearBeltsMotorIdleMode = CANSparkBase.IdleMode.kBrake

    kBeltsIntakeSpeed: float = 0.6
    kBeltsAdjustmentSpeed: float = 0.4
    kBeltsEjectSpeed: float = 0.6
    kBeltsLaunchSpeed: float = 0.6

    kIntakeTriggerDistanceIn: float = 180.0
    kIntakeTriggerDistanceOut: float = 320.0
    kLauncherTriggerDistanceIn: float = 160.0
    kLauncherTargetDistanceMin: float = 45.0
    kLauncherTargetDistanceMax: float = 105.0

    kReloadDelay: units.seconds = 0.2

  class Launcher:
    kArmMotorCANId: int = 11
    kBottomRollerMotorCANId: int = 12
    kTopRollerMotorCANId: int = 13

    kArmMotorCurrentLimit: int = 60
    kArmMotorMaxReverseOutput: float = -1.0
    kArmMotorMaxForwardOutput: float = 1.0
    kArmMotorIdleMode = CANSparkBase.IdleMode.kBrake
    kArmMotorPIDConstants = PIDConstants(0.0003, 0, 0.00015, 1 / 16.8)
    kArmMotorForwardSoftLimit: float = 12.0
    kArmMotorReverseSoftLimit: float = 1.0
    kArmMotorPositionConversionFactor: float = 1.0 / 3.0
    kArmMotorVelocityConversionFactor: float = kArmMotorPositionConversionFactor / 60.0
    kArmMotorSmartMotionMaxVelocity: float = (33.0 / kArmMotorPositionConversionFactor) * 60
    kArmMotorSmartMotionMaxAccel: float = 100.0 / kArmMotorVelocityConversionFactor

    kBottomRollerMotorCurrentLimit = 100
    kBottomRollerMotorMaxForwardOutput: float = 1.0
    kBottomRollerMotorMaxReverseOutput: float = -1.0
    kBottomRollerMotorIdleMode = CANSparkBase.IdleMode.kCoast

    kTopRollerMotorCurrentLimit = 100
    kTopRollerMotorMaxForwardOutput: float = 1.0
    kTopRollerMotorMaxReverseOutput: float = -1.0
    kTopRollerMotorIdleMode = CANSparkBase.IdleMode.kCoast

    kRollersLaunchStartDelay: units.seconds = 0.75

    kRollersSpeedsSpeaker = LauncherRollersSpeeds(0.8, 0.8)
    kRollersSpeedsAmp = LauncherRollersSpeeds(0.27, 0.27)
    kRollersSpeedsShuttle = LauncherRollersSpeeds(0.6, 0.6)
    kRollersSpeedsWarmup = LauncherRollersSpeeds(0.8, 0.8)

    kArmInputLimiter: float = 0.5
    kArmTargetAlignmentPositionTolerance: float = 0.1
    
    kArmPositionSubwoofer: float = 9.8
    kArmPositionPodium: float = 3.6
    kArmPositionAmp: float = 9.5
    kArmPositionShuttle: float = 9.5 # TODO: recalibrate with on-field testing
    kArmPositionClimber: float = 1.0
    kArmPositionIntake: float = 2.0

    # TODO: recalibrate with on-field testing
    kArmPositionTargets: list[LauncherArmPositionTarget] = [
      LauncherArmPositionTarget(1.0, 10.2),
      LauncherArmPositionTarget(1.6, 9.8),
      LauncherArmPositionTarget(2.45, 5.4),
      LauncherArmPositionTarget(3.15, 3.5),
      LauncherArmPositionTarget(4.25, 1.6),
      LauncherArmPositionTarget(5.45, 1.4),
      LauncherArmPositionTarget(6.55, 1.2),
      LauncherArmPositionTarget(7.65, 1.1)
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

    kArmPositionDefault: float = 8.2

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
      "Left": Transform3d(
        Translation3d(units.inchesToMeters(5.75), units.inchesToMeters(3.25), units.inchesToMeters(14.0)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-23.5), units.degreesToRadians(90))
      ),
      "Right": Transform3d(
        Translation3d(units.inchesToMeters(-3.25), units.inchesToMeters(-11.5), units.inchesToMeters(15.5)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-28.4), units.degreesToRadians(-90.0))
      ),
      "Rear": Transform3d(
        Translation3d(units.inchesToMeters(-4.75), units.inchesToMeters(-11.25), units.inchesToMeters(20.0)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-24.0), units.degreesToRadians(180.0))
      )
    }
    kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kVisionSingleTagStandardDeviations: tuple[float, ...] = [1.0, 1.0, 2.0]
    kVisionMultiTagStandardDeviations: tuple[float, ...] = [0.5, 0.5, 1.0]
    kVisionMaxPoseAmbiguity: float = 0.2

  class Distance:
    class Intake:
      kSensorName = "Intake"
      kMinTargetDistance: float = 0
      kMaxTargetDistance: float = 320
    class Launcher:
      kSensorName = "Launcher"
      kMinTargetDistance: float = 0
      kMaxTargetDistance: float = 320
    class Climber:
      kSensorName = "Climber"
      kMinTargetDistance: float = 0
      kMaxTargetDistance: float = 240

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
      AutoPath.Test: PathPlannerPath.fromPathFile(AutoPath.Test.name),
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
