package frc.robot.subsystems;

import javax.sound.sampled.DataLine;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
//import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class LiftArm extends SubsystemBase {
  private final CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.kLeftArmMotorCANID, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.kRightArmMotorCANID, MotorType.kBrushless);

  private final AbsoluteEncoder liftEncoder;
  private final RelativeEncoder liftRelativeEncoder;
  double requestedSpeed = 0;

  private final SparkMaxPIDController m_PIDController;

  private DataLog armLog;
  private DoubleLogEntry armSetpoint;

  public double setpoint;

  public LiftArm(){
    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();
    
    liftEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    liftRelativeEncoder= leftArmMotor.getEncoder();

    m_PIDController = leftArmMotor.getPIDController();
    m_PIDController.setFeedbackDevice(liftEncoder);

    // Probably dont need this
    m_PIDController.setPositionPIDWrappingEnabled(false);

    m_PIDController.setP(ArmConstants.kP);
    m_PIDController.setI(ArmConstants.kI);
    m_PIDController.setD(ArmConstants.kD);
    m_PIDController.setFF(ArmConstants.kFF);

    // Temporary values
    m_PIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

  

    leftArmMotor.setOpenLoopRampRate(ArmConstants.kRampRate);
    rightArmMotor.setOpenLoopRampRate(ArmConstants.kRampRate);

    liftRelativeEncoder.setPositionConversionFactor(ArmConstants.kConversionFactor);
    liftEncoder.setPositionConversionFactor(ArmConstants.kConversionFactor);

    leftArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
    rightArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);

    // Set relative encoder position to absolute encoder position * 72ticks/rotation
    // TODO: Fix this
    liftRelativeEncoder.setPosition(liftEncoder.getPosition() * ArmConstants.kGearing);

    // TODO: Fix this
    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.kEnableForwardLimit);
    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.kEnableReverseLimit);
    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.kEnableForwardLimit);
    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.kEnableReverseLimit);
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)ArmConstants.kForwardLimit);
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)ArmConstants.kReverseLimit);


    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    
    // Set right motor to follow left, inverted
    rightArmMotor.follow(leftArmMotor, true);

    leftArmMotor.setInverted(true);

    leftArmMotor.burnFlash();
    rightArmMotor.burnFlash();

    this.setpoint = liftEncoder.getPosition();
    SmartDashboard.putNumber("Arm setpoint", this.setpoint);

    armLog = DataLogManager.getLog();
    armSetpoint = new DoubleLogEntry(armLog, "/setpoint");

  }

  // Set speed of arm
  public void move(double speed){
    requestedSpeed = speed;
    leftArmMotor.set(speed);
  }

  // Sets setpoint, where setpoint is 0 to 1
  public void setPosition(double setpoint) {
    this.setpoint = setpoint;
    m_PIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  // Gets absolute position
  public double getPosition() {
    return liftEncoder.getPosition();
  }

  @Override
  // Logging
  public void periodic() {
    SmartDashboard.putNumber("Relative location", liftRelativeEncoder.getPosition());
    SmartDashboard.putNumber("Absolute location", liftEncoder.getPosition());
    SmartDashboard.putNumber("Applied Speed", rightArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("Desired Speeed", requestedSpeed);
    SmartDashboard.putNumber("Arm setpoint", setpoint);
    liftRelativeEncoder.setPosition(this.getPosition());

    armSetpoint.append(setpoint);
  }
}
