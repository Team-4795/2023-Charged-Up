package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
    // private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private static Wrist mInstance;

    public static Wrist getInstance() {
        if (mInstance == null) {
            switch (Constants.getRobot()) {
                case Comp:
                    // mInstance = new Wrist(new WristIOReal());
                    break;
                case Sim:
                    // mInstance = new Wrist(new WristIO() {});
                    break;
            }
        }

        return mInstance;
    }

    public boolean extendedTarget = false;
    public boolean extended = false;

    private Timer extensionTimer;
    private double timerOffset = 0.0;

    public final double angleChangeDeg = 98.205;
    public final double extensionTime = 0.15;

    public Wrist() {
        // this.io = io;

        extensionTimer = new Timer();
        extensionTimer.start();
    }

    private void extend() {
        if (!extended) {
            timerOffset = Math.max(extensionTime - extensionTimer.get() - timerOffset, 0.0);
            extensionTimer.reset();
        }
        extended = true;
        // io.set(Value.kForward);
    }

    private void retract() {
        if (extended) {
            timerOffset = Math.max(extensionTime - extensionTimer.get() - timerOffset, 0.0);
            extensionTimer.reset();
        }
        extended = false;
        // io.set(Value.kReverse);
    }

    public void flip() {
        setExtendedTarget(!this.extendedTarget);
    }

    public void setExtendedTarget(boolean extend) {
        this.extendedTarget = extend;
    }

    public double getEstimatedAngleDeg() {
        if (extended) {
            return MathUtil.interpolate(0, angleChangeDeg, (extensionTimer.get() + timerOffset) / extensionTime);
        } else {
            return MathUtil.interpolate(angleChangeDeg, 0, (extensionTimer.get() + timerOffset) / extensionTime);
        }
    }

    public double getSetpointDeg() {
        if (extended) {
            return angleChangeDeg;
        } else {
            return 0;
        }
    }

    public void constrain(double arm) {
        double armPos = Arm.getInstance().getPosition();

        if (armPos < ArmConstants.kLowWristLimit || armPos > ArmConstants.kHighWristLimit) {
            retract();
        } else {
            if (extendedTarget) {
                extend();
            } else {
                retract();
            }
        }
    }

    @Override
    public void periodic() {
        // io.updateInputs(inputs);
        Logger.getInstance().processInputs("Wrist", inputs);

        constrain(Arm.getInstance().getPosition());

        Logger.getInstance().recordOutput("Wrist estimated angle", getEstimatedAngleDeg());
    }
}
