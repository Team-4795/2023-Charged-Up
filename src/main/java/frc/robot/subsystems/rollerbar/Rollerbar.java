// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollerbar;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RollerbarConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class Rollerbar extends SubsystemBase {
    private final RollerbarIO io;
    private final RollerbarIOInputsAutoLogged inputs = new RollerbarIOInputsAutoLogged();

    private static Rollerbar mInstance;

    public static Rollerbar getInstance() {
        if (mInstance == null) {
            switch (Constants.getRobot()) {
                case Comp:
                    mInstance = new Rollerbar(new RollerbarIOReal());
                    break;
                case Sim:
                    mInstance = new Rollerbar(new RollerbarIO() {});
                    break;
            }
        }

        return mInstance;
    }

    // Set to its respective extension once rollerbar has begun moving
    private boolean movingToExtended;

    // Target extension
    private boolean targetExtend;

    // Fully extended or retracted
    private boolean extended;

    private RollerbarVisualizer viz = new RollerbarVisualizer("Setpoint", Color.kOrange);

    private Timer extensionTimer;
    private double timerOffset = 0.0;

    public final double startAngle = 96.189;
    public final double endAngle = 22.353;
    public final double extensionTime = 0.4;

    public Rollerbar(RollerbarIO io) {
        this.io = io;

        movingToExtended = getExtension();
        targetExtend = getExtension();
        extended = getExtension();

        extensionTimer = new Timer();
        extensionTimer.start();

        setDefaultCommand(run(() -> {
            if (!Intake.isStoring() && isExtended() && Arm.getInstance().atSetpoint()) {
                spin();
            } else {
                stop();
            }
        }));
    }

    public boolean shouldMove() {
        return extended != targetExtend;
    }

    public boolean safeToMove(double position) {
        return position > RollerbarConstants.kArmBoundary && position < RollerbarConstants.kDoubleExtensionBoundary;
    }

    public boolean isExtended() {
        return extended;
    }

    public void setExtendedTarget(boolean extend) {
        targetExtend = extend;
    }

    public void spin() {
        io.setSpin(RollerbarConstants.kSpinSpeed);
    }

    public void stop() {
        io.setSpin(0);
    }

    public void reverse() {
        io.setSpin(0.8);
    }

    private double getMoveTime() {
        if (extended) {
            return RollerbarConstants.kRetractTime;
        } else {
            return RollerbarConstants.kExtendTime;
        }
    }

    public double getEstimatedAngleDeg() {
        if (movingToExtended) {
            return MathUtil.interpolate(startAngle, endAngle, (extensionTimer.get() + timerOffset) / extensionTime);
        } else {
            return MathUtil.interpolate(endAngle, startAngle, (extensionTimer.get() + timerOffset) / extensionTime);
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Rollerbar", inputs);

        viz.update(getEstimatedAngleDeg());

        // Set `extended` on a delay to account for physical movement time
        if (extensionTimer.hasElapsed(getMoveTime())) {
            extended = movingToExtended;
        }

        // Move arm if possible
        if (safeToMove(Arm.getInstance().getPosition())) {
            move();
        }
    }

    private boolean getExtension() {
        switch (io.getExtension()) {
            case kForward:
                return true;
            default:
                return false;
        }
    }

    public void extend() {
        if (!movingToExtended) {
            timerOffset = Math.max(extensionTime - extensionTimer.get() - timerOffset, 0.0);
            extensionTimer.reset();
        }
        movingToExtended = true;
        io.setExtension(Value.kForward);
    }

    public void retract() {
        if (movingToExtended) {
            timerOffset = Math.max(extensionTime - extensionTimer.get() - timerOffset, 0.0);
            extensionTimer.reset();
        }
        movingToExtended = false;
        io.setExtension(Value.kReverse);
    }

    // Force move to target
    private void move() {
        if (targetExtend) {
            extend();
        } else {
            retract();
        }
    }

    // Force toggle
    public void toggle() {
        if (movingToExtended) {
            retract();
        } else {
            extend();
        }
    }
}
