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

  def periodic(self) -> None:
    self._updatePose()
    self._updateTelemetry()

  def getPose(self) -> Pose2d:
    return self._poseEstimator.getEstimatedPosition()

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

  def getTargetPose(self) -> Pose3d:
    return utils.getValueForAlliance(
      constants.Game.Field.Targets.kBlueSpeaker, 
      constants.Game.Field.Targets.kRedSpeaker
    )

  def getTargetYaw(self) -> float:
    robotPose = self.getPose()
    targetPose = self.getTargetPose().toPose2d().transformBy(
      Transform2d(
        utils.getValueForAlliance(
          constants.Game.Field.Targets.kSpeakerTargetYawTransformX, 
          -constants.Game.Field.Targets.kSpeakerTargetYawTransformX
        ), 
        constants.Game.Field.Targets.kSpeakerTargetYTransform, 
        Rotation2d.fromDegrees(0)
      )
    )
    targetTranslation = targetPose.relativeTo(robotPose).translation()
    targetRotation = Rotation2d(targetTranslation.X(), targetTranslation.Y()).rotateBy(Rotation2d.fromDegrees(180)).rotateBy(robotPose.rotation())
    return utils.wrapAngle(targetRotation.degrees())
  
  def getTargetPitch(self) -> float:
    return utils.getPitchToPose(
      Pose3d(self.getPose()),
      self.getTargetPose().transformBy(
        Transform3d(0, 0, constants.Game.Field.Targets.kSpeakerTargetPitchTransformZ, Rotation3d())
      )
    )
  
  def getTargetDistance(self) -> float:
    return utils.getDistanceToPose(
      self.getPose(),
      self.getTargetPose().toPose2d().transformBy(
        Transform2d(
          utils.getValueForAlliance(
            -constants.Game.Field.Targets.kSpeakerTargetDistanceTransformX, 
            constants.Game.Field.Targets.kSpeakerTargetDistanceTransformX
          ), 0, Rotation2d.fromDegrees(0)
        )
      )
    )

  def _updateTelemetry(self) -> None:
    robotPose = self.getPose()
    SmartDashboard.putNumberArray("Robot/Pose/Current", [robotPose.X(), robotPose.Y(), robotPose.rotation().degrees()])
    SmartDashboard.putNumber("Robot/Pose/Target/Yaw", self.getTargetYaw())
    SmartDashboard.putNumber("Robot/Pose/Target/Pitch", self.getTargetPitch())
    SmartDashboard.putNumber("Robot/Pose/Target/Distance", self.getTargetDistance())