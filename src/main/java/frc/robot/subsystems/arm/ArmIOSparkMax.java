package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Constants.ArmConstants;

public class ArmIOSparkMax implements ArmIO {
    private final CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.kLeftArmMotorCANID, MotorType.kBrushless);
    private final CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.kRightArmMotorCANID, MotorType.kBrushless);

    private final AbsoluteEncoder liftEncoder;

    public ArmIOSparkMax() {
        rightArmMotor.restoreFactoryDefaults();

        liftEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);

        leftArmMotor.setOpenLoopRampRate(ArmConstants.kRampRate);
        rightArmMotor.setOpenLoopRampRate(ArmConstants.kRampRate);
        leftArmMotor.setClosedLoopRampRate(ArmConstants.kRampRate);
        rightArmMotor.setClosedLoopRampRate(ArmConstants.kRampRate);

        leftArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        rightArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);

        leftArmMotor.setIdleMode(IdleMode.kBrake);
        rightArmMotor.setIdleMode(IdleMode.kBrake);

        rightArmMotor.follow(leftArmMotor, true);

        leftArmMotor.setInverted(true);
        leftArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        leftArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
        leftArmMotor.burnFlash();
        rightArmMotor.burnFlash();
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.angleRev = liftEncoder.getPosition();
        inputs.angleRevPerSec = liftEncoder.getVelocity ();
        inputs.appliedVolts = leftArmMotor.getAppliedOutput() * leftArmMotor.getBusVoltage();
    }

    public void setArmVoltage(double volts) {
        leftArmMotor.setVoltage(volts);
    }
}
