package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.IntakeConstants;

public class WristIOReal implements WristIO {
    private final DoubleSolenoid solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, IntakeConstants.kForwardChannel, IntakeConstants.kReverseChannel);
    private final PneumaticHub m_ph = new PneumaticHub(IntakeConstants.kPHCANID);
    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    public WristIOReal() {
        compressor.enableAnalog(IntakeConstants.kMinPressure, IntakeConstants.kMaxPressure);
    }

    public void updateInputs(WristIOInputs inputs) {
        inputs.pressure = m_ph.getPressure(0);
    }

    public void set(Value value) {
        solenoid.set(value);
    }
}
