package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class LiftArm extends SubsystemBase {
  private final CANSparkMax leftArmMotor = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor = new CANSparkMax(11, MotorType.kBrushless);

  private final AbsoluteEncoder liftEncoder;
  private final RelativeEncoder liftRelativeEncoder;
  double requestedSpeed = 0;

  private final SparkMaxPIDController m_PIDController;

  public LiftArm(){
    
    liftEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    liftRelativeEncoder= leftArmMotor.getEncoder();

    m_PIDController = leftArmMotor.getPIDController();
    m_PIDController.setFeedbackDevice(liftEncoder);

    // Probably dont need this
    m_PIDController.setPositionPIDWrappingEnabled(false);

    m_PIDController.setP(0.05);
    m_PIDController.setI(0);
    m_PIDController.setD(0);
    m_PIDController.setFF(0);

    // Temporary values
    m_PIDController.setOutputRange(-0.25, 0.25);

    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    leftArmMotor.setOpenLoopRampRate(0.5);
    rightArmMotor.setOpenLoopRampRate(0.5);

    liftRelativeEncoder.setPositionConversionFactor(1);
    liftEncoder.setPositionConversionFactor(1);

    leftArmMotor.setSmartCurrentLimit(60);
    rightArmMotor.setSmartCurrentLimit(60);

    // Set relative encoder position to absolute encoder position * 72ticks/rotation
    // TODO: Fix this
    liftRelativeEncoder.setPosition(liftEncoder.getPosition() * 72);

    // TODO: Fix this
    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 65);
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 10);


    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    
    // Set right motor to follow left, inverted
    rightArmMotor.follow(leftArmMotor, true);

    leftArmMotor.setInverted(true);

    leftArmMotor.burnFlash();
    rightArmMotor.burnFlash();
  }

  // Set speed of arm
  public void move(double speed){
    requestedSpeed = speed;
    leftArmMotor.set(speed);
  }

  // // Sets setpoint, where setpoint is in encoder ticks (position converion factor is 1.0)
  public void setPosition(double setpoint){
    m_PIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // Logging
    SmartDashboard.putNumber("Relative location", liftRelativeEncoder.getPosition());
    SmartDashboard.putNumber("Absolute location", liftEncoder.getPosition());
    SmartDashboard.putNumber("Applied Speed", rightArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("Desired Speeed", requestedSpeed);
  }
}
