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

  // private final SparkMaxPIDController m_PIDController;

  public LiftArm(){

    liftEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    liftRelativeEncoder= leftArmMotor.getEncoder();

    // m_PIDController = leftArmMotor.getPIDController();

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
  
  if(speed>0.25)speed=0.25; 
  if(speed<-0.25)speed=-0.25; 

  leftArmMotor.set(speed);
 }


public void resetEncoders() {
  liftRelativeEncoder.setPosition(0);
}

  @Override
  public void periodic() {}

}
