package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiftArm extends ProfiledPIDSubsystem {
  private final CANSparkMax leftArmMotor = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor = new CANSparkMax(11, MotorType.kBrushless);

  private final Encoder liftEncoder = new Encoder(0,1);

 // PIDController m_pid = new PIDController(0, 0, 0);

  private final ArmFeedforward m_feedforward = new ArmFeedforward(0, -0.04, 0, 0);

  public LiftArm(){

    super(new ProfiledPIDController(0,0,0, new TrapezoidProfile.Constraints(1,1)),0);// change

    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();
    leftArmMotor.setSmartCurrentLimit(30);
    rightArmMotor.setSmartCurrentLimit(30);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setInverted(false); // do not twist
    leftArmMotor.setInverted(true);
    rightArmMotor.follow(leftArmMotor);
   
    
    liftEncoder.setDistancePerPulse(0.01388888888); // not zero change
    
  }

  public void move(double speed){

    leftArmMotor.setVoltage(speed);
    // leftArmMotor.setVoltage(m_feedforward.calculate(, speed));
    //if(liftEncoder.getDistance() > (double)1/60 && output + feedforward > 0) leftArmMotor.setVoltage(0);

   //if(liftEncoder.getDistance() > 0 && output + feedforward < 0) leftArmMotor.setVoltage(0);
 }

 @Override
 public void useOutput(double output, TrapezoidProfile.State setpoint) {
 
  double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
  leftArmMotor.setVoltage(output + feedforward);

  if(liftEncoder.getDistance() > (double)1/60 && output + feedforward > 0) leftArmMotor.setVoltage(0);

  if(liftEncoder.getDistance() > 0 && output + feedforward < 0) leftArmMotor.setVoltage(0);

}

@Override
public double getMeasurement() {
  return liftEncoder.getDistance() + 0; // some constant here
}

  public void resetEncoder() {
    liftEncoder.reset();
  }


  @Override
  public void periodic() {}

}

//
