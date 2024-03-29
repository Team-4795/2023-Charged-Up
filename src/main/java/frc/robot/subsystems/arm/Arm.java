package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RollerbarConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.StateManager;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollerbar.Rollerbar;
import frc.robot.subsystems.motorizedWrist.Wrist;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private static Arm mInstance;

    public static Arm getInstance() {
        if (mInstance == null) {
            switch (Constants.getRobot()) {
                case Comp:
                    mInstance = new Arm(new ArmIOSparkMax());
                    break;
                case Sim:
                    mInstance = new Arm(new ArmIOSim());
                    break;
            }
        }

        return mInstance;
    }

    private double setpoint;

    private ArmController controller;

    public ArmVisualizer armVisualizerMeasured = new ArmVisualizer("Measured", Color.kOrange);
    public ArmVisualizer armVisualizerSetpoint = new ArmVisualizer("Setpoint", Color.kRed);

    // Set when the arm is moving to avoid rollerbar or double extension
    private boolean isTemporary = false;

    private Arm(ArmIO io) {
        this.io = io;
        io.updateInputs(inputs);

        setTargetPosition(getPosition());

        controller = new ArmController(getPosition());

        setDefaultCommand(run(() -> {
            double up = MathUtil.applyDeadband(
                    OIConstants.operatorController.getRightTriggerAxis(), OIConstants.kArmDeadband);
            double down = MathUtil.applyDeadband(
                    OIConstants.operatorController.getLeftTriggerAxis(), OIConstants.kArmDeadband);

            // Get amount to change the setpoint
            double change = OIConstants.kArmManualSpeed * (Math.pow(up, 3) - Math.pow(down, 3));

            setTargetPosition(setpoint + change);
        }));
    }

    private Constraints getConstraints() {
        if (!Intake.isStoring()) {
            return ArmConstants.kNotStoringConstraint;
        } else if (StateManager.getInstance().getGamepiece() == StateManager.Gamepiece.Cone) {
            return ArmConstants.kConeMotionConstraint;
        } else {
            return ArmConstants.kCubeMotionConstraint;
        }
    }

    // Sets setpoint, where setpoint is 0 to 1
    public void setTargetPosition(double newSetpoint) {
        setpoint = MathUtil.clamp(newSetpoint, ArmConstants.kLowSetpointLimit, ArmConstants.kHighSetpointLimit);
    }

    // Gets absolute position, in revolutions
    public double getPosition() {
        return inputs.angleRev;
    }

    public double getAngleDeg() {
        return inputs.angleRev * 360 - 90;
    }

    public double getSetpointDeg() {
        return setpoint * 360 - 90;
    }

    public double getSetpoint(){
        return setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(getPosition() - setpoint) < ArmConstants.kPositionThreshold && !isTemporary;
    }

    public void resetPosition() {
        this.setTargetPosition(this.getPosition());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);

        // armVisualizerMeasured.update(getAngleDeg(), Wrist.getInstance().getAngleDeg());
        // armVisualizerSetpoint.update(getSetpointDeg(), Wrist.getInstance().getSetpointDeg());

        Logger.getInstance().recordOutput("Arm setpoint", setpoint);

        double feedback = controller.calculate(inputs.angleRev, setpoint, getConstraints());
        io.setArmVoltage(feedback);
    }
}
