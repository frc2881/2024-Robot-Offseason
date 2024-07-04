import math
from wpilib import SmartDashboard
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonCamera import PhotonCamera
from lib import utils

class ObjectSensor:
  def __init__(
      self, 
      cameraName: str,
      objectName: str
    ) -> None:
    self._cameraName = cameraName
    self._objectName = objectName

    self._baseKey = f'Robot/Sensor/Object/{self._cameraName}/{self._objectName}'
    self._photonCamera = PhotonCamera(cameraName)
    self._hasTarget = False

    utils.addRobotPeriodic(self._updateTelemetry)

  def getTargetInfo(self) -> PhotonTrackedTarget:
    if self._photonCamera.isConnected():
      result = self._photonCamera.getLatestResult()
      if result.hasTargets():
        self._hasTarget = True
        return result.getTargets()[0]
    self._hasTarget = False
    return PhotonTrackedTarget()

  def hasTarget(self) -> bool:
    return self._hasTarget

  def _updateTelemetry(self) -> None:
    targetInfo = self.getTargetInfo()
    SmartDashboard.putBoolean(f'{self._baseKey}/IsConnected', self._photonCamera.isConnected())
    SmartDashboard.putBoolean(f'{self._baseKey}/HasTarget', self.hasTarget())
    SmartDashboard.putNumber(f'{self._baseKey}/Target/Heading', targetInfo.getYaw())
    SmartDashboard.putNumber(f'{self._baseKey}/Target/Area', targetInfo.getArea())
