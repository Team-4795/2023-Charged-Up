package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class LiftArm extends ProfiledPIDSubsystem {
  private final CANSparkMax leftArmMotor = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor = new CANSparkMax(1, MotorType.kBrushless);

  private final Encoder liftEncoder = new Encoder(0,0);

 // PIDController m_pid = new PIDController(0, 0, 0);

  private final ArmFeedforward m_feedforward = new ArmFeedforward(0, -0.04, 0, 0);

  public LiftArm(){
    super(new ProfiledPIDController(0,0,0, new TrapezoidProfile.Constraints(0,0)),0);// change

    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setInverted(true);
    leftArmMotor.setInverted(true);
    rightArmMotor.follow(leftArmMotor);

    
    liftEncoder.setDistancePerPulse(0); // not zero change
  }

  public void move(double speed){
    leftArmMotor.setVoltage(speed);
    // leftArmMotor.setVoltage(m_feedforward.calculate(, speed));

 }

 @Override
 public void useOutput(double output, TrapezoidProfile.State setpoint) {
  // Calculate the feedforward from the sepoint
  double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
  leftArmMotor.setVoltage(output + feedforward);
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
