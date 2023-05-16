package frc.robot.subsystems.rollerbar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.RollerbarConstants;

public class RollerbarIOReal implements RollerbarIO {
    private final DoubleSolenoid solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, RollerbarConstants.kForwardChannel, RollerbarConstants.kReverseChannel);

    private final CANSparkMax rollerMotor = new CANSparkMax(RollerbarConstants.kRollerbarCANID, MotorType.kBrushed);

    public RollerbarIOReal() {
        rollerMotor.restoreFactoryDefaults();
        rollerMotor.setIdleMode(IdleMode.kBrake);
        rollerMotor.setSmartCurrentLimit(30);
        rollerMotor.burnFlash();
    }

    public void updateInputs(RollerbarIOInputs inputs) {
        inputs.appliedOutput = rollerMotor.getAppliedOutput();
        inputs.currentAmps = rollerMotor.getOutputCurrent();
    }

    public void setExtension(Value value) {
        solenoid.set(value);
    }

    public void getExtension(Value value) {
        solenoid.get();
    }

    public void setSpin(double speed) {
        rollerMotor.set(speed);
    }
}
