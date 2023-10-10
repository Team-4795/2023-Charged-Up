package frc.robot.subsystems.motorizedWrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WristConstants;

public class WristIOReal implements WristIO {
    
    private CANSparkMax wristMotor = new CANSparkMax(WristConstants.CANID, MotorType.kBrushless);
    private AbsoluteEncoder encoder;
    private RelativeEncoder relative;

    public WristIOReal(){
        encoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        relative = wristMotor.getEncoder();
        relative.setPosition(0);
        relative.setPositionConversionFactor(0.04166667);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
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
        inputs.angle = relative.getPosition();
        inputs.angularVelocity = encoder.getVelocity();
        inputs.motorVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
        inputs.current = wristMotor.getOutputCurrent();
        SmartDashboard.putNumber("Wrist Absolute Encoder", encoder.getPosition());
    }

    @Override
    public void set(double motorSpeed) {
        wristMotor.set(motorSpeed);
    }
    
}
