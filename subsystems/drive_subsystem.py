from typing import Callable
import math
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, SendableChooser
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics
from rev import CANSparkBase
from lib import utils, logger
from lib.classes import SwerveModuleLocation, DriveSpeedMode, DriveOrientation, DriveDriftCorrection, DriveLockState
from lib.components.swerve_module import SwerveModule
import constants

class DriveSubsystem(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], float]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Drive

    self._getGyroHeading: Callable[[], float] = getGyroHeading

    self._swerveModuleFrontLeft = SwerveModule(
      SwerveModuleLocation.FrontLeft,
      self._constants.kSwerveModuleFrontLeftDrivingMotorCANId,
      self._constants.kSwerveModuleFrontLeftTurningMotorCANId,
      self._constants.kSwerveModuleFrontLeftOffset,
      self._constants.SwerveModule 
    )

    self._swerveModuleFrontRight = SwerveModule(
      SwerveModuleLocation.FrontRight,
      self._constants.kSwerveModuleFrontRightDrivingMotorCANId,
      self._constants.kSwerveModuleFrontRightTurningMotorCANId,
      self._constants.kSwerveModuleFrontRightOffset,
      self._constants.SwerveModule
    )

    self._swerveModuleRearLeft = SwerveModule(
      SwerveModuleLocation.RearLeft,
      self._constants.kSwerveModuleRearLeftDrivingMotorCANId,
      self._constants.kSwerveModuleRearLeftTurningMotorCANId,
      self._constants.kSwerveModuleRearLeftOffset,
      self._constants.SwerveModule
    )

    self._swerveModuleRearRight = SwerveModule(
      SwerveModuleLocation.RearRight,
      self._constants.kSwerveModuleRearRightDrivingMotorCANId,
      self._constants.kSwerveModuleRearRightTurningMotorCANId,
      self._constants.kSwerveModuleRearRightOffset,
      self._constants.SwerveModule
    )

    self._isDriftCorrectionActive: bool = False
    self._driftCorrectionThetaController = PIDController(
      self._constants.kDriftCorrectionThetaControllerPIDConstants.P, 
      self._constants.kDriftCorrectionThetaControllerPIDConstants.I, 
      self._constants.kDriftCorrectionThetaControllerPIDConstants.D
    )
    self._driftCorrectionThetaController.enableContinuousInput(-180.0, 180.0)
    self._driftCorrectionThetaController.setTolerance(
      self._constants.kDriftCorrectionThetaControllerPositionTolerance, 
      self._constants.kDriftCorrectionThetaControllerVelocityTolerance
    )

    self._isAlignedToTarget: bool = False
    self._targetAlignmentThetaController = PIDController(
      self._constants.kTargetAlignmentThetaControllerPIDConstants.P, 
      self._constants.kTargetAlignmentThetaControllerPIDConstants.I, 
      self._constants.kTargetAlignmentThetaControllerPIDConstants.D
    )
    self._targetAlignmentThetaController.enableContinuousInput(-180.0, 180.0)
    self._targetAlignmentThetaController.setTolerance(
      self._constants.kTargetAlignmentThetaControllerPositionTolerance, 
      self._constants.kTargetAlignmentThetaControllerVelocityTolerance
    )

    self._driveInputXFilter = SlewRateLimiter(self._constants.kDriveInputRateLimit)
    self._driveInputYFilter = SlewRateLimiter(self._constants.kDriveInputRateLimit)
    self._driveInputRotFilter = SlewRateLimiter(self._constants.kDriveInputRateLimit)

    self._speedMode: DriveSpeedMode = DriveSpeedMode.Competition
    driveSpeedModeChooser = SendableChooser()
    driveSpeedModeChooser.setDefaultOption(DriveSpeedMode.Competition.name, DriveSpeedMode.Competition)
    driveSpeedModeChooser.addOption(DriveSpeedMode.Training.name, DriveSpeedMode.Training)
    driveSpeedModeChooser.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    SmartDashboard.putData("Robot/Drive/SpeedMode", driveSpeedModeChooser)

    self._orientation: DriveOrientation = DriveOrientation.Field
    driveOrientationChooser = SendableChooser()
    driveOrientationChooser.setDefaultOption(DriveOrientation.Field.name, DriveOrientation.Field)
    driveOrientationChooser.addOption(DriveOrientation.Robot.name, DriveOrientation.Robot)
    driveOrientationChooser.onChange(lambda orientation: setattr(self, "_orientation", orientation))
    SmartDashboard.putData("Robot/Drive/Orientation", driveOrientationChooser)

    self._driftCorrection: DriveDriftCorrection = DriveDriftCorrection.Enabled
    driveDriftCorrectionChooser = SendableChooser()
    driveDriftCorrectionChooser.setDefaultOption(DriveDriftCorrection.Enabled.name, DriveDriftCorrection.Enabled)
    driveDriftCorrectionChooser.addOption(DriveDriftCorrection.Disabled.name, DriveDriftCorrection.Disabled)
    driveDriftCorrectionChooser.onChange(lambda driftCorrection: setattr(self, "_driftCorrection", driftCorrection))
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driveDriftCorrectionChooser)

    idleModeChooser = SendableChooser()
    idleModeChooser.setDefaultOption(CANSparkBase.IdleMode.kBrake.name.lstrip("k"), CANSparkBase.IdleMode.kBrake)
    idleModeChooser.addOption(CANSparkBase.IdleMode.kCoast.name.lstrip("k"), CANSparkBase.IdleMode.kCoast)
    idleModeChooser.onChange(lambda idleMode: self._setIdleMode(idleMode))
    SmartDashboard.putData("Robot/Drive/IdleMode", idleModeChooser)

    self._lockState: DriveLockState = DriveLockState.Unlocked

  def periodic(self) -> None:
    self._updateTelemetry()

  def driveWithControllerCommand(
      self, 
      getLeftY: Callable[[], float], 
      getLeftX: Callable[[], float], 
      getRightX: Callable[[], float]
    ) -> Command:
    return self.run(
      lambda: self._driveWithController(getLeftY(), getLeftX(), getRightX())
    ).onlyIf(
      lambda: self._lockState != DriveLockState.Locked
    ).withName("DriveSubsystem:DriveWithController")

  def _driveWithController(self, speedX: float, speedY: float, speedRotation: float) -> None:
    if self._driftCorrection == DriveDriftCorrection.Enabled:
      isTranslating: bool = speedX != 0 or speedY != 0
      isRotating: bool = speedRotation != 0
      if isTranslating and not isRotating and not self._isDriftCorrectionActive:
        self._isDriftCorrectionActive = True
        self._driftCorrectionThetaController.reset()
        self._driftCorrectionThetaController.setSetpoint(self._getGyroHeading())
      elif isRotating or not isTranslating:
        self._isDriftCorrectionActive = False
      if self._isDriftCorrectionActive:
        speedRotation: float = self._driftCorrectionThetaController.calculate(self._getGyroHeading())
        if self._driftCorrectionThetaController.atSetpoint():
          speedRotation = 0

    if self._speedMode == DriveSpeedMode.Training:
      speedX: float = self._driveInputXFilter.calculate(speedX * self._constants.kDriveInputLimiter)
      speedY: float = self._driveInputYFilter.calculate(speedY * self._constants.kDriveInputLimiter)
      speedRotation: float = self._driveInputRotFilter.calculate(speedRotation * self._constants.kDriveInputLimiter)

    self._drive(speedX, speedY, speedRotation)      

  def _drive(self, speedX: float, speedY: float, speedRotation: float) -> None:
    speedX *= self._constants.kMaxSpeedMetersPerSecond
    speedY *= self._constants.kMaxSpeedMetersPerSecond
    speedRotation *= self._constants.kMaxAngularSpeed
    if self._orientation == DriveOrientation.Field:
      self.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedRotation, Rotation2d.fromDegrees(self._getGyroHeading())))
    else:
      self.drive(ChassisSpeeds(speedX, speedY, speedRotation))

  def drive(self, chassisSpeeds: ChassisSpeeds) -> None:
    self._setSwerveModuleStates(
      self._constants.kSwerveDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(chassisSpeeds, 0.02)
      )
    )

  def _setSwerveModuleStates(self, swerveModuleStates: tuple[SwerveModuleState, ...]) -> None:
    SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, self._constants.kMaxSpeedMetersPerSecond)
    frontLeft, frontRight, rearLeft, rearRight = swerveModuleStates
    self._swerveModuleFrontLeft.setTargetState(frontLeft)
    self._swerveModuleFrontRight.setTargetState(frontRight)
    self._swerveModuleRearLeft.setTargetState(rearLeft)
    self._swerveModuleRearRight.setTargetState(rearRight)

  def getSpeeds(self) -> ChassisSpeeds:
    return self._constants.kSwerveDriveKinematics.toChassisSpeeds(self._getSwerveModuleStates())

  def getSwerveModulePositions(self) -> tuple[SwerveModulePosition, ...]:
    return (
      self._swerveModuleFrontLeft.getPosition(),
      self._swerveModuleFrontRight.getPosition(),
      self._swerveModuleRearLeft.getPosition(),
      self._swerveModuleRearRight.getPosition()
    )
  
  def _getSwerveModuleStates(self) -> tuple[SwerveModuleState, ...]:
    return (
      self._swerveModuleFrontLeft.getState(),
      self._swerveModuleFrontRight.getState(),
      self._swerveModuleRearLeft.getState(),
      self._swerveModuleRearRight.getState()
    )
  
  def _setIdleMode(self, idleMode: CANSparkBase.IdleMode):
    self._swerveModuleFrontLeft.setIdleMode(idleMode)
    self._swerveModuleFrontRight.setIdleMode(idleMode)
    self._swerveModuleRearLeft.setIdleMode(idleMode)
    self._swerveModuleRearRight.setIdleMode(idleMode)
    SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name.lstrip("k"))

  def lockCommand(self) -> Command:
    return self.startEnd(
      lambda: self._setLockState(DriveLockState.Locked),
      lambda: self._setLockState(DriveLockState.Unlocked)
    ).withName("DriveSubsystem:Lock")
  
  def _setLockState(self, lockState: DriveLockState) -> None:
    self._lockState = lockState
    if lockState == DriveLockState.Locked:
      self._swerveModuleFrontLeft.setTargetState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
      self._swerveModuleFrontRight.setTargetState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
      self._swerveModuleRearLeft.setTargetState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
      self._swerveModuleRearRight.setTargetState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

  def alignToTargetCommand(self, getRobotPose: Callable[[], Pose2d], getTargetYaw: Callable[[], float]) -> Command:
    return self.run(
      lambda: self._alignToTarget(getRobotPose().rotation().degrees(), getTargetYaw())
    ).beforeStarting(
      lambda: [
        self.clearTargetAlignment(),
        self._targetAlignmentThetaController.setSetpoint(getTargetYaw()),
        self._targetAlignmentThetaController.reset()
      ]
    ).onlyIf(
      lambda: self._lockState != DriveLockState.Locked
    ).until(
      lambda: self._isAlignedToTarget
    ).withName("DriveSubsystem:AlignToTarget")

  def _alignToTarget(self, robotYaw: float, targetYaw: float) -> None:
    speedRotation: float = self._targetAlignmentThetaController.calculate(robotYaw)
    speedRotation += math.copysign(0.15, speedRotation)
    if self._targetAlignmentThetaController.atSetpoint():
      speedRotation = 0
      self._isAlignedToTarget = True
    self._setSwerveModuleStates(
      self._constants.kSwerveDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds(0, 0, speedRotation)
      )
    )

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def reset(self) -> None:
    self._setIdleMode(CANSparkBase.IdleMode.kBrake)
    self._drive(0, 0, 0)
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putString("Robot/Drive/LockState", self._lockState.name)
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTarget", self._isAlignedToTarget)
  