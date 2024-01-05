package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;

public class DeadbandXboxController extends CommandXboxController {
    public DeadbandXboxController(int port) {
        super(port);
    }

    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(getLeftX(), 0.1);
    }

    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(getRightX(), 0.1);
    }

    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(getLeftY(), 0.1);
    }

    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(getRightY(), 0.1);
    }

    @Override
    public double getLeftTriggerAxis() {
        return MathUtil.applyDeadband(getLeftTriggerAxis(), OIConstants.kArmDeadband);
    }

    @Override
    public double getRightTriggerAxis() {
        return MathUtil.applyDeadband(getRightTriggerAxis(), OIConstants.kArmDeadband);
    }

    public double getLeftMagnitude() {
        return MathUtil.applyDeadband(Math.hypot(getLeftX(), getLeftY()), 0.1);
    }

    public double getRightMagnitude() {
        return MathUtil.applyDeadband(Math.hypot(getRightX(), getRightY()), 0.1);
    }
}
