package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LandingGearConstants;

public class LandingGear extends SubsystemBase {
    private static LandingGear mInstance;

    public static LandingGear getInstance() {
        if (mInstance == null) {
            mInstance = new LandingGear();
        }

        return mInstance;
    }

    private final DoubleSolenoid solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, LandingGearConstants.kForwardChannel, LandingGearConstants.kBackwardChannel);
    // placeholders, change later
    private boolean extended;
    private boolean targetExtended;

    private LandingGear() {
        extended = false;
        targetExtended = false;

        setDefaultCommand(run(() -> {
            if (getExtended() != getTargetExtended()) {
                updateExtended();
            }
        }));
    }

    private void land() {
        solenoid.set(Value.kForward);
        extended = true;
    }

    private void retract() {
        solenoid.set(Value.kReverse);
        extended = false;
    }

    public void setTargetExtended(boolean target) {
        targetExtended = target;
    }

    public boolean getExtended() {
        return extended;
    }

    public boolean getTargetExtended() {
        return targetExtended;
    }

    public void updateExtended() {
        if (targetExtended) {
            this.land();
        } else {
            this.retract();
        }
    }
}
