package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeCANID, MotorType.kBrushed);

    public IntakeIOSparkMax() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        intakeMotor.enableVoltageCompensation(12);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);
        intakeMotor.burnFlash();
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedOutput = intakeMotor.getAppliedOutput();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
    }

    public void setOutput(double output) {
        // intakeMotor.setVoltage(output * 12);
        intakeMotor.set(output);
    }
}
