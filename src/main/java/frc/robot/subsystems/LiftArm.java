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

  private final SparkMaxPIDController m_PIDController;

  public LiftArm(){

    liftEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    liftRelativeEncoder= leftArmMotor.getEncoder();

    m_PIDController = leftArmMotor.getPIDController();
    m_PIDController.setFeedbackDevice(liftEncoder);

    m_PIDController.setP(0.05);
    m_PIDController.setI(0);
    m_PIDController.setD(0);
    m_PIDController.setFF(0);

    // Temporary values
    m_PIDController.setOutputRange(-0.25, 0.25);

    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    liftRelativeEncoder.setPositionConversionFactor(1);
    liftEncoder.setPositionConversionFactor(1);

    leftArmMotor.setSmartCurrentLimit(60);
    rightArmMotor.setSmartCurrentLimit(60);

    // Set relative encoder position to absolute encoder position
    liftRelativeEncoder.setPosition(liftEncoder.getPosition());

    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 55);
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 55);
    rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    rightArmMotor.setInverted(false); 
    leftArmMotor.setInverted(true);

    rightArmMotor.follow(leftArmMotor);    

    leftArmMotor.burnFlash();
    rightArmMotor.burnFlash();
  }

  public void move(double speed){
    leftArmMotor.set(speed);
  }

  // Sets setpoint, where setpoint is in encoder ticks (position converion factor is 1.0)
  public void setPosition(double setpoint){
    m_PIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {}

}
