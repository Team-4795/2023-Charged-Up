package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Wrist extends SubsystemBase{
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.kForwardChannel, IntakeConstants.kReverseChannel);
    private final PneumaticHub m_ph = new PneumaticHub(IntakeConstants.kPHCANID);
    private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    public boolean extendedTarget = false;
    public boolean extended = false;
    int x = 0;

    public Wrist(){
        compressor.enableAnalog(IntakeConstants.kMinPressure, IntakeConstants.kMaxPressure);
    }

    public void extend() {
        extended = true;
        solenoid.set(Value.kForward);
    }

    public void retract() {
        extended = false;
        solenoid.set(Value.kReverse);
    }

    public void flip() {
        x++;
        setExtendedTarget(!this.extendedTarget);
        SmartDashboard.putNumber("flip", x);
    }

    public void setExtendedTarget(boolean extend) {
        this.extendedTarget = extend;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pressure", m_ph.getPressure(0));
        SmartDashboard.putBoolean("Wrist extended target", extendedTarget);
        SmartDashboard.putBoolean("Wrist extended", extended);
    }

}
