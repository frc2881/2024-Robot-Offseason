from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard, Servo
from rev import CANSparkBase, CANSparkLowLevel, CANSparkMax
from lib import utils, logger
import constants

class ClimberSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    
    self._constants = constants.Subsystems.Climber

    self._hasInitialZeroReset: bool = False

    self._armLeftMotor = CANSparkMax(self._constants.kArmLeftMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._armLeftEncoder = self._armLeftMotor.getEncoder()
    self._armLeftPIDController = self._armLeftMotor.getPIDController()
    utils.validateParam(self._armLeftMotor.restoreFactoryDefaults())
    utils.validateParam(self._armLeftMotor.setIdleMode(self._constants.kArmMotorIdleMode))
    utils.validateParam(self._armLeftMotor.setSmartCurrentLimit(self._constants.kArmMotorCurrentLimit))
    utils.validateParam(self._armLeftMotor.setSecondaryCurrentLimit(self._constants.kArmMotorCurrentLimit))
    utils.validateParam(self._armLeftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, True))
    utils.validateParam(self._armLeftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, self._constants.kArmMotorForwardSoftLimit)) 
    utils.validateParam(self._armLeftMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, True))
    utils.validateParam(self._armLeftMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, self._constants.kArmMotorReverseSoftLimit))
    self._armLeftMotor.setInverted(True)
    utils.validateParam(self._armLeftPIDController.setP(self._constants.kArmMotorPIDConstants.P))
    utils.validateParam(self._armLeftPIDController.setD(self._constants.kArmMotorPIDConstants.D))
    utils.validateParam(self._armLeftPIDController.setOutputRange(self._constants.kArmMotorMaxReverseOutput, self._constants.kArmMotorMaxForwardOutput))
    utils.validateParam(self._armLeftMotor.burnFlash())

    self._armRightMotor = CANSparkMax(self._constants.kArmRightMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._armRightEncoder = self._armRightMotor.getEncoder()
    utils.validateParam(self._armRightMotor.restoreFactoryDefaults())
    utils.validateParam(self._armRightMotor.setIdleMode(self._constants.kArmMotorIdleMode))
    utils.validateParam(self._armRightMotor.setSmartCurrentLimit(self._constants.kArmMotorCurrentLimit))
    utils.validateParam(self._armRightMotor.setSecondaryCurrentLimit(self._constants.kArmMotorCurrentLimit))
    utils.validateParam(self._armRightMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, True))
    utils.validateParam(self._armRightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, self._constants.kArmMotorForwardSoftLimit)) 
    utils.validateParam(self._armRightMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, True))
    utils.validateParam(self._armRightMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, self._constants.kArmMotorReverseSoftLimit))
    utils.validateParam(self._armRightMotor.follow(self._armLeftMotor, True))
    utils.validateParam(self._armRightMotor.burnFlash())

    self._brakeServo = Servo(self._constants.kBrakeServoChannel)

  def periodic(self) -> None:
    self._updateTelemetry()

  def moveArmUpCommand(self) -> Command:
    return self.startEnd(
      lambda: self._armLeftMotor.set(0.5),
      lambda: self._armLeftMotor.set(0)
    ).withName("ClimberSubsystem:MoveArmUp")
  
  def moveArmDownCommand(self) -> Command:
    return self.startEnd(
      lambda: self._armLeftMotor.set(-0.5),
      lambda: self._armLeftMotor.set(0)
    ).withName("ClimberSubsystem:MoveArmDown")

  def moveArmToDefaultPositionCommand(self) -> Command:
    return self.runOnce(
      lambda: self._armLeftPIDController.setReference(self._constants.kArmPositionDefault, CANSparkLowLevel.ControlType.kPosition)
    ).withName("ClimberSubsystem:MoveArmToDefaultPosition")

  def unlockArmCommand(self) -> Command:
    return cmd.runOnce(
      lambda: self._brakeServo.set(1.0)
    ).withName("ClimberSubsystem:UnlockArm")

  def lockArmCommand(self) -> Command:
    return cmd.runOnce(
      lambda: self._brakeServo.set(0)
    ).withName("ClimberSubsystem:LockArm")

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        utils.enableSoftLimits(self._armLeftMotor, False),
        utils.enableSoftLimits(self._armRightMotor, False),
        self._armLeftMotor.set(-0.1)
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
    self._brakeServo.set(1.0)
    self._armLeftPIDController.setReference(self._constants.kArmPositionDefault, CANSparkBase.ControlType.kPosition)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Climber/Arm/Position", self._armLeftEncoder.getPosition())
    SmartDashboard.putNumber("Robot/Climber/Brake/Position", self._brakeServo.get())