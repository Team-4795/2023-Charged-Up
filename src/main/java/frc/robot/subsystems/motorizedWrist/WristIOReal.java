package frc.robot.subsystems.motorizedWrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.WristConstants;

public class WristIOReal implements WristIO {
    
    private CANSparkMax wristMotor = new CANSparkMax(WristConstants.CANID, MotorType.kBrushless);
    private AbsoluteEncoder encoder;

    public WristIOReal(){
        encoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

        wristMotor.setOpenLoopRampRate(0);
        wristMotor.setClosedLoopRampRate(0);
        wristMotor.setSmartCurrentLimit(0);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.burnFlash();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.angle = encoder.getPosition();
        inputs.angularVelocity = encoder.getVelocity();
        inputs.motorVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    }

    @Override
    public void set(double motorSpeed) {
        wristMotor.set(motorSpeed);
    }
    
}
