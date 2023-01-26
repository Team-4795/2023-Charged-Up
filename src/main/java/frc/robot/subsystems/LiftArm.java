package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftArm extends SubsystemBase {
  private final CANSparkMax liftArmMotor = new CANSparkMax(0, MotorType.kBrushless);
  private final RelativeEncoder liftEncoder = liftArmMotor.getEncoder();;

  public LiftArm(){

    liftArmMotor.restoreFactoryDefaults();
    liftArmMotor.setIdleMode(IdleMode.kBrake);
    liftArmMotor.setInverted(true);

 // all below does not work need spark alternative and is not working
 // need more research on how to implement PID

   
  }

  public void move(double speed){
    liftArmMotor.set(speed);
 }

  public void resetEncoder() {
    liftEncoder.setPosition(0);
  }


  @Override
  public void periodic() {}

}

//
