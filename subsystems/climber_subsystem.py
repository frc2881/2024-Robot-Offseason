from wpilib import SmartDashboard, Servo
from commands2 import Subsystem, Command, cmd
from rev import CANSparkBase, CANSparkLowLevel, CANSparkMax
from lib import utils, logger
import constants

class ClimberSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    
    self._constants = constants.Subsystems.Climber

    self._hasInitialZeroReset: bool = False

    self._armLeftMotor = CANSparkMax(self._constants.Arm.kLeftMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._armLeftEncoder = self._armLeftMotor.getEncoder()
    self._armLeftPIDController = self._armLeftMotor.getPIDController()
    utils.validateParam(self._armLeftMotor.restoreFactoryDefaults())
    utils.validateParam(self._armLeftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._armLeftMotor.setSmartCurrentLimit(self._constants.Arm.kMotorCurrentLimit))
    utils.validateParam(self._armLeftMotor.setSecondaryCurrentLimit(self._constants.Arm.kMotorCurrentLimit))
    utils.validateParam(self._armLeftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, True))
    utils.validateParam(self._armLeftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, self._constants.Arm.kMotorForwardSoftLimit)) 
    utils.validateParam(self._armLeftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, True))
    utils.validateParam(self._armLeftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, self._constants.Arm.kMotorReverseSoftLimit))
    self._armLeftMotor.setInverted(True)
    utils.validateParam(self._armLeftPIDController.setP(self._constants.Arm.kMotorPIDConstants.P))
    utils.validateParam(self._armLeftPIDController.setD(self._constants.Arm.kMotorPIDConstants.D))
    utils.validateParam(self._armLeftPIDController.setOutputRange(self._constants.Arm.kMotorMaxReverseOutput, self._constants.Arm.kMotorMaxForwardOutput))
    utils.validateParam(self._armLeftMotor.burnFlash())

    self._armRightMotor = CANSparkMax(self._constants.Arm.kRightMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._armRightEncoder = self._armRightMotor.getEncoder()
    utils.validateParam(self._armRightMotor.restoreFactoryDefaults())
    utils.validateParam(self._armRightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._armRightMotor.setSmartCurrentLimit(self._constants.Arm.kMotorCurrentLimit))
    utils.validateParam(self._armRightMotor.setSecondaryCurrentLimit(self._constants.Arm.kMotorCurrentLimit))
    utils.validateParam(self._armRightMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, True))
    utils.validateParam(self._armRightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, self._constants.Arm.kMotorForwardSoftLimit)) 
    utils.validateParam(self._armRightMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, True))
    utils.validateParam(self._armRightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, self._constants.Arm.kMotorReverseSoftLimit))
    utils.validateParam(self._armRightMotor.follow(self._armLeftMotor, True))
    utils.validateParam(self._armRightMotor.burnFlash())

    self._brakeServo = Servo(self._constants.Brake.kServoChannel)

  def periodic(self) -> None:
    self._updateTelemetry()

  def moveArmUpCommand(self) -> Command:
    return self.run(
      lambda: self._armLeftMotor.set(self._constants.Arm.kInputLimit)
    ).finallyDo(
      lambda: self._armLeftMotor.set(0)
    ).withName("ClimberSubsystem:MoveArmUp")
  
  def moveArmDownCommand(self) -> Command:
    return self.run(
      lambda: self._armLeftMotor.set(-self._constants.Arm.kInputLimit)
    ).finallyDo(
      lambda: self._armLeftMotor.set(0)
    ).withName("ClimberSubsystem:MoveArmDown")

  def setArmToDefaultPositionCommand(self) -> Command:
    return self.runOnce(
      lambda: self._setToDefaultPosition()
    ).withName("ClimberSubsystem:SetArmToDefaultPosition")

  def unlockArmCommand(self) -> Command:
    return cmd.runOnce(
      lambda: self._brakeServo.set(self._constants.Brake.kPositionUnlocked)
    ).withName("ClimberSubsystem:UnlockArm")

  def lockArmCommand(self) -> Command:
    return cmd.runOnce(
      lambda: self._brakeServo.set(self._constants.Brake.kPositionLocked)
    ).withName("ClimberSubsystem:LockArm")
  
  def _setToDefaultPosition(self) -> None:
    self._armLeftPIDController.setReference(self._constants.Arm.kPositionDefault, CANSparkLowLevel.ControlType.kPosition)

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        utils.enableSoftLimits(self._armLeftMotor, False),
        utils.enableSoftLimits(self._armRightMotor, False),
        self._armLeftMotor.set(-self._constants.Arm.kResetSpeed)
      ],
      lambda: [
        self._armLeftEncoder.setPosition(0),
        self._armRightEncoder.setPosition(0),
        self._armLeftMotor.set(0),
        utils.enableSoftLimits(self._armLeftMotor, True),
        utils.enableSoftLimits(self._armRightMotor, True),
        setattr(self, "_hasInitialZeroReset", True)
      ]
    ).withName("ClimberSubsystem:ResetArmToZero")

  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroReset

  def reset(self) -> None:
    self._armLeftMotor.set(0)
    self._brakeServo.set(self._constants.Brake.kPositionUnlocked)
    self._setToDefaultPosition()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Climber/Arm/Position", self._armLeftEncoder.getPosition())
    SmartDashboard.putNumber("Robot/Climber/Brake/Position", self._brakeServo.get())