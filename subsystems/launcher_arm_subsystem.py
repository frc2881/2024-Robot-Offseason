from typing import Callable
import math
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import CANSparkBase, CANSparkLowLevel, CANSparkMax
from lib import utils, logger
import constants

class LauncherArmSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    
    self._constants = constants.Subsystems.Launcher.Arm

    self._hasInitialZeroReset: bool = False
    self._isAlignedToTarget: bool = False
    self._targetDistances = list(map(lambda p: p.distance, self._constants.kPositionTargets))
    self._targetPositions = list(map(lambda p: p.position, self._constants.kPositionTargets))
    
    SmartDashboard.putString("Robot/Launcher/Arm/Positions", utils.toJson(self._constants.kPositionTargets))

    self._armMotor = CANSparkMax(self._constants.kMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._armEncoder = self._armMotor.getEncoder()
    self._armPIDController = self._armMotor.getPIDController()
    utils.validateParam(self._armMotor.restoreFactoryDefaults())
    utils.validateParam(self._armMotor.setIdleMode(self._constants.kMotorIdleMode))
    utils.validateParam(self._armMotor.setSmartCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._armMotor.setSecondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    utils.validateParam(self._armMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, True))
    utils.validateParam(self._armMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, self._constants.kMotorForwardSoftLimit))
    utils.validateParam(self._armMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, True))
    utils.validateParam(self._armMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, self._constants.kMotorReverseSoftLimit))
    utils.validateParam(self._armEncoder.setPositionConversionFactor(self._constants.kMotorPositionConversionFactor))
    utils.validateParam(self._armEncoder.setVelocityConversionFactor(self._constants.kMotorVelocityConversionFactor))
    utils.validateParam(self._armPIDController.setFeedbackDevice(self._armEncoder))
    utils.validateParam(self._armPIDController.setP(self._constants.kMotorPIDConstants.P))
    utils.validateParam(self._armPIDController.setD(self._constants.kMotorPIDConstants.D))
    utils.validateParam(self._armPIDController.setOutputRange(self._constants.kMotorMaxReverseOutput, self._constants.kMotorMaxForwardOutput))
    utils.validateParam(self._armPIDController.setSmartMotionMaxVelocity(self._constants.kMotorSmartMotionMaxVelocity, 0))
    utils.validateParam(self._armPIDController.setSmartMotionMaxAccel(self._constants.kMotorSmartMotionMaxAccel, 0))
    utils.validateParam(self._armMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def runCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._armMotor.set(getInput() * self._constants.kInputLimit)
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("LauncherArmSubsystem:Run")

  def alignToPositionCommand(self, position: float) -> Command:
    return self.run(
      lambda: [
        self._armPIDController.setReference(position, CANSparkBase.ControlType.kSmartMotion),
        self._setIsAlignedToTarget(position)
      ]
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("LauncherArmSubsystem:AlignToPosition")
  
  def alignToTargetCommand(self, getTargetDistance: Callable[[], units.meters]) -> Command:
    return self.run(
      lambda: [
        position := self._getTargetPosition(getTargetDistance()),
        self._armPIDController.setReference(position, CANSparkBase.ControlType.kSmartMotion),
        self._setIsAlignedToTarget(position)
      ]
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("LauncherArmSubsystem:AlignToTarget")

  def _getTargetPosition(self, targetDistance: units.meters) -> float:
    targetPosition = utils.getInterpolatedValue(targetDistance, self._targetDistances, self._targetPositions)
    if utils.isValueInRange(targetPosition, self._constants.kMotorReverseSoftLimit, self._constants.kMotorForwardSoftLimit):
      return targetPosition
    else:
      return self._constants.kPositionSubwoofer

  def _setIsAlignedToTarget(self, position: float) -> None:
    self._isAlignedToTarget = math.fabs(self._armEncoder.getPosition() - position) <= self._constants.kTargetAlignmentPositionTolerance

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        utils.enableSoftLimits(self._armMotor, False),
        self._armMotor.set(-self._constants.kResetSpeed)
      ],
      lambda: [
        self._armEncoder.setPosition(0),
        self._armMotor.set(0),
        utils.enableSoftLimits(self._armMotor, True),
        setattr(self, "_hasInitialZeroReset", True)
      ]
    ).withName("LauncherArmSubsystem:ResetToZero")

  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroReset

  def reset(self) -> None:
    self._armMotor.set(0)
    self.clearTargetAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Launcher/Arm/Position", self._armEncoder.getPosition())
    SmartDashboard.putBoolean("Robot/Launcher/Arm/IsAlignedToTarget", self._isAlignedToTarget)