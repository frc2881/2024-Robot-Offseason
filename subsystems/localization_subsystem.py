from typing import Callable
from commands2 import Subsystem
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d, Pose3d, Transform2d, Transform3d, Rotation3d
from wpimath.kinematics import SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib.sensors.pose_sensor import PoseSensor
from lib import utils, logger
import constants

class LocalizationSubsystem(Subsystem):
  def __init__(
      self,
      poseSensors: list[PoseSensor],
      getGyroRotation: Callable[[], Rotation2d],
      getSwerveModulePositions: Callable[[], tuple[SwerveModulePosition, ...]]
    ) -> None:
    super().__init__()
    self._poseSensors = poseSensors
    self._getGyroRotation = getGyroRotation
    self._getSwerveModulePositions = getSwerveModulePositions

    self._poseEstimator = SwerveDrive4PoseEstimator(
      constants.Subsystems.Drive.kSwerveDriveKinematics,
      self._getGyroRotation(),
      self._getSwerveModulePositions(),
      Pose2d()
    )

    self._currentAlliance = None
    self._targetPose = Pose3d()
    self._targetHeading: float = 0.0
    self._targetPitch: float = 0.0
    self._targetDistance: float = 0.0

  def periodic(self) -> None:
    self._updatePose()
    self._updateTargetPose()
    self._updateTargetInfo()
    self._updateTelemetry()

  def _updatePose(self) -> None:
    self._poseEstimator.update(self._getGyroRotation(), self._getSwerveModulePositions())
    for poseSensor in self._poseSensors:
      estimatedRobotPose = poseSensor.getEstimatedRobotPose()
      if estimatedRobotPose is not None:
        pose = estimatedRobotPose.estimatedPose.toPose2d()
        if self._isPoseOnField(pose):
          if estimatedRobotPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR:
            self._poseEstimator.addVisionMeasurement(
              pose,
              estimatedRobotPose.timestampSeconds,
              constants.Sensors.Pose.kVisionMultiTagStandardDeviations
            )
          else:
            for target in estimatedRobotPose.targetsUsed:
              if utils.isValueInRange(target.getPoseAmbiguity(), 0, constants.Sensors.Pose.kVisionMaxPoseAmbiguity):
                self._poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds, constants.Sensors.Pose.kVisionSingleTagStandardDeviations)
                break

  def getPose(self) -> Pose2d:
    return self._poseEstimator.getEstimatedPosition()

  def resetPose(self, pose: Pose2d) -> None:
    # NO-OP as current pose is always maintained by pose sensors in the configuration for this robot
    # self._poseEstimator.resetPosition(self._getGyroRotation(), self._getSwerveModulePositions(), pose)
    pass   

  def _isPoseOnField(self, pose: Pose2d) -> bool:
    x: float = pose.X()
    y: float = pose.Y()
    return (
      (x >= 0 and x <= constants.Game.Field.kAprilTagFieldLayout.getFieldLength()) 
      and 
      (y >= 0 and y <= constants.Game.Field.kAprilTagFieldLayout.getFieldWidth())
    )

  def hasVisionTargets(self) -> bool:
    for poseSensor in self._poseSensors:
      if poseSensor.hasTarget():
        return True
    return False

  def _updateTargetPose(self) -> None:
    if utils.getAlliance() != self._currentAlliance:
      self._currentAlliance = utils.getAlliance()
      self._targetPose = utils.getValueForAlliance(
        constants.Game.Field.Targets.kBlueSpeaker, 
        constants.Game.Field.Targets.kRedSpeaker
      ).transformBy(
        constants.Game.Field.Targets.kSpeakerTargetTransform
      )

  def _updateTargetInfo(self) -> None:
    robotPose = Pose3d(self.getPose())
    self._targetDistance = utils.getDistanceToPose(robotPose, self._targetPose)
    self._targetHeading = utils.getHeadingToPose(robotPose, self._targetPose)
    self._targetPitch = utils.getPitchToPose(robotPose, self._targetPose)

  def getTargetDistance(self) -> float:
    return self._targetDistance

  def getTargetHeading(self) -> float:
    return self._targetHeading
  
  def getTargetPitch(self) -> float:
    return self._targetPitch
  
  def _updateTelemetry(self) -> None:
    robotPose = self.getPose()
    SmartDashboard.putNumberArray("Robot/Localization/Pose", [robotPose.X(), robotPose.Y(), robotPose.rotation().degrees()])
    SmartDashboard.putNumber("Robot/Localization/Target/Distance", self.getTargetDistance())
    SmartDashboard.putNumber("Robot/Localization/Target/Heading", self.getTargetHeading())
    SmartDashboard.putNumber("Robot/Localization/Target/Pitch", self.getTargetPitch())
