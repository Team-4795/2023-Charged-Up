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
  private final SparkMaxPIDController test;

  public LiftArm(){

    liftArmMotor.restoreFactoryDefaults();
    liftArmMotor.setIdleMode(IdleMode.kBrake);
    liftArmMotor.setInverted(true);

 // all below does not work need spark alternative and is not working
 // need more research on how to implement PID

    test = new SparkMaxPIDController(liftArmMotor);

    liftArmMotor.configVoltageCompSaturation(12);
    liftArmMotor.enableVoltageCompensation(true);   

    liftArmMotor.configSelectedFeedbackSensor(, 0, 0);

    liftArmMotor.config_kF(0, 0, 0);
    liftArmMotor.config_kP(0, 0, 0);
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
