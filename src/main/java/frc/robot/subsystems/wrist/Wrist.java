package frc.robot.subsystems.wrist;

import javax.swing.text.html.parser.Entity;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.arm.Arm;

public class Wrist extends SubsystemBase {
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.kForwardChannel, IntakeConstants.kReverseChannel);
    private final PneumaticHub m_ph = new PneumaticHub(IntakeConstants.kPHCANID);
    private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    private Timer extensionTimer;

    public boolean extendedTarget = false;
    public boolean extended = false;

    public static Pose3d wristPose = new Pose3d();

    public Wrist(){
        compressor.enableAnalog(IntakeConstants.kMinPressure, IntakeConstants.kMaxPressure);

        extensionTimer = new Timer();
        extensionTimer.start();
    }

    public void extend() {
        if (!extended) {
            extensionTimer.reset();
        }
        extended = true;
        solenoid.set(Value.kForward);
    }

    public void retract() {
        if (extended) {
            extensionTimer.reset();
        }
        extended = false;
        solenoid.set(Value.kReverse);
    }

    public void flip() {
        setExtendedTarget(!this.extendedTarget);
    }

    public void setExtendedTarget(boolean extend) {
        this.extendedTarget = extend;
    }

    public double estimateAngleRad() {
        if (extended) {
            return MathUtil.interpolate(0, 1.714, 3*extensionTimer.get());
        } else {
            return MathUtil.interpolate(1.714, 0, 3*extensionTimer.get());
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pressure", m_ph.getPressure(0));
        SmartDashboard.putBoolean("Wrist extended target", extendedTarget);
        SmartDashboard.putBoolean("Wrist extended", extended);
        SmartDashboard.putNumber("Wrist time", extensionTimer.get());
        wristPose = new Pose3d(0.5334, 0.0, 0.0, new Rotation3d(0, -estimateAngleRad(), 0));
    }

}
