package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ArmConstants;

public class ArmIOSparkMax implements ArmIO {
  private final CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.kLeftArmMotorCANID, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.kRightArmMotorCANID, MotorType.kBrushless);

  private final AbsoluteEncoder liftEncoder;
  private final RelativeEncoder liftRelativeEncoder;

  public ArmIOSparkMax() {
    rightArmMotor.restoreFactoryDefaults();
    
    liftEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    liftRelativeEncoder = leftArmMotor.getEncoder();

    leftArmMotor.setOpenLoopRampRate(ArmConstants.kRampRate);
    rightArmMotor.setOpenLoopRampRate(ArmConstants.kRampRate);
    leftArmMotor.setClosedLoopRampRate(ArmConstants.kRampRate);
    rightArmMotor.setClosedLoopRampRate(ArmConstants.kRampRate);

    leftArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
    rightArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);

    liftRelativeEncoder.setPosition(liftEncoder.getPosition() * ArmConstants.kGearing);

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    
    // Set right motor to follow left, inverted
    rightArmMotor.follow(leftArmMotor, true);

    leftArmMotor.setInverted(true);

    leftArmMotor.burnFlash();
    rightArmMotor.burnFlash();
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.armAngleRev = liftEncoder.getPosition();
    inputs.armAngleRev = liftEncoder.getVelocity();
  }

  public void setArmVoltage(double volts) {
    leftArmMotor.setVoltage(volts);
  }
}
