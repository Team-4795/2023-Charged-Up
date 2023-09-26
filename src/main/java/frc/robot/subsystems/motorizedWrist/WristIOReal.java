package frc.robot.subsystems.motorizedWrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.WristConstants;

public class WristIOReal implements WristIO {
    
    private CANSparkMax wristMotor = new CANSparkMax(WristConstants.CANID, MotorType.kBrushless);
    private AbsoluteEncoder encoder;

    public WristIOReal(){
        encoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);
        wristMotor.setOpenLoopRampRate(0.3);
        wristMotor.setClosedLoopRampRate(0.3);
        wristMotor.setSmartCurrentLimit(WristConstants.currentLimit);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.burnFlash();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.angle = encoder.getPosition();
        inputs.angularVelocity = encoder.getVelocity();
        inputs.motorVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
        inputs.current = wristMotor.getOutputCurrent();
    }

    @Override
    public void set(double motorSpeed) {
        wristMotor.set(motorSpeed);
    }
    
}
