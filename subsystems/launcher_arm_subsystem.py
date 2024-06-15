from typing import Callable
import math
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from rev import CANSparkBase, CANSparkLowLevel, CANSparkMax
from lib import utils, logger
import constants

class LauncherArmSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    
    self._constants = constants.Subsystems.Launcher

    self._hasInitialZeroReset: bool = False
    self._isAlignedToTarget: bool = False
    self._targetDistances = list(map(lambda p: p.distance, self._constants.kArmPositionTargets))
    self._targetPositions = list(map(lambda p: p.position, self._constants.kArmPositionTargets))
    
    SmartDashboard.putString("Robot/Launcher/Arm/Positions", utils.toJson(self._constants.kArmPositionTargets))

    self._armMotor = CANSparkMax(self._constants.kArmMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._armEncoder = self._armMotor.getEncoder()
    self._armPIDController = self._armMotor.getPIDController()
    utils.validateParam(self._armMotor.restoreFactoryDefaults())
    utils.validateParam(self._armMotor.setIdleMode(self._constants.kArmMotorIdleMode))
    utils.validateParam(self._armMotor.setSmartCurrentLimit(self._constants.kArmMotorCurrentLimit))
    utils.validateParam(self._armMotor.setSecondaryCurrentLimit(self._constants.kArmMotorCurrentLimit))
    utils.validateParam(self._armMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, True))
    utils.validateParam(self._armMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, self._constants.kArmMotorForwardSoftLimit))
    utils.validateParam(self._armMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, True))
    utils.validateParam(self._armMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, self._constants.kArmMotorReverseSoftLimit))
    utils.validateParam(self._armEncoder.setPositionConversionFactor(self._constants.kArmMotorPositionConversionFactor))
    utils.validateParam(self._armEncoder.setVelocityConversionFactor(self._constants.kArmMotorVelocityConversionFactor))
    utils.validateParam(self._armPIDController.setFeedbackDevice(self._armEncoder))
    utils.validateParam(self._armPIDController.setP(self._constants.kArmMotorPIDConstants.P))
    utils.validateParam(self._armPIDController.setD(self._constants.kArmMotorPIDConstants.D))
    utils.validateParam(self._armPIDController.setOutputRange(self._constants.kArmMotorMaxReverseOutput, self._constants.kArmMotorMaxForwardOutput))
    utils.validateParam(self._armPIDController.setSmartMotionMaxVelocity(self._constants.kArmMotorSmartMotionMaxVelocity, 0))
    utils.validateParam(self._armPIDController.setSmartMotionMaxAccel(self._constants.kArmMotorSmartMotionMaxAccel, 0))
    utils.validateParam(self._armMotor.burnFlash())

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def runCommand(self, getSpeed: Callable[[], float]) -> Command:
    return self.run(
      lambda: self._armMotor.set(getSpeed() * self._constants.kArmInputLimiter)
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("LauncherArmSubsystem:RunLauncherArm")

  def alignToPositionCommand(self, position: float) -> Command:
    return self.run(
      lambda: [
        self._armPIDController.setReference(position, CANSparkBase.ControlType.kSmartMotion),
        self._setIsAlignedToTarget(position)
      ]
    ).until(
      lambda: utils.isAutonomousMode() and self.isAlignedToTarget()
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("LauncherArmSubsystem:AlignToPosition")
  
  def alignToTargetCommand(self, getTargetDistance: Callable[[], float]) -> Command:
    return self.run(
      lambda: [
        position := self._getTargetPosition(getTargetDistance()),
        self._armPIDController.setReference(position, CANSparkBase.ControlType.kSmartMotion),
        self._setIsAlignedToTarget(position)
      ]
    ).until(
      lambda: utils.isAutonomousMode() and self.isAlignedToTarget()
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("LauncherArmSubsystem:AlignToTarget")

  def getPosition(self) -> float:
    return self._armEncoder.getPosition()

  def _getTargetPosition(self, targetDistance: float) -> float:
    targetPosition = utils.getInterpolatedValue(self._targetDistances, self._targetPositions, targetDistance)
    if utils.isValueInRange(targetPosition, self._constants.kArmMotorReverseSoftLimit, self._constants.kArmMotorForwardSoftLimit):
      return targetPosition
    else:
      return self._constants.kArmPositionSubwoofer

  def _setIsAlignedToTarget(self, position: float) -> None:
    self._isAlignedToTarget = math.fabs(self._armEncoder.getPosition() - position) <= self._constants.kArmTargetAlignmentPositionTolerance

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        utils.enableSoftLimits(self._armMotor, False),
        self._armMotor.set(-0.1)
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

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Launcher/Arm/Position", self.getPosition())
    SmartDashboard.putBoolean("Robot/Launcher/Arm/IsAlignedToTarget", self.isAlignedToTarget())