package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.1, 0.1);

    private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0, 0.02);
    private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0, 0.02);

    public SwerveModuleState setpoints = new SwerveModuleState(0.0, new Rotation2d());

    private int index;

    public Module(ModuleIO io, int index) {
        this.index = index;
        this.io = io;

        // switch (index) {
        //   case 0:
        //     chassisAngularOffset = DriveConstants.kFrontLeftChassisAngularOffset;
        //     break;
        //   case 1:
        //     chassisAngularOffset = DriveConstants.kFrontRightChassisAngularOffset;
        //     break;
        //   case 2:
        //     chassisAngularOffset = DriveConstants.kBackLeftChassisAngularOffset;
        //     break;
        //   case 3:
        //     chassisAngularOffset = DriveConstants.kBackRightChassisAngularOffset;
        //     break;
        //   default:
        //     throw new RuntimeException("Invalid module index for Module");
        // }

        driveFeedback.setPID(0.9, 0.0, 0.0);

        // driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.kDrivingFF, 0.0);

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
        turnFeedback.setPID(23, 0.0, 0.0);
    }

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module" + Integer.toString(index), inputs);
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        // Optimize state based on current angle
        var optimizedState = SwerveModuleState.optimize(state, getAngle());

        // Run turn controller
        io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

        // Update velocity based on turn error
        optimizedState.speedMetersPerSecond *= Math.cos(turnFeedback.getPositionError());

        // Run drive controller
        double velocityRadPerSec = optimizedState.speedMetersPerSecond / ModuleConstants.kWheelRadiusMeters;
        io.setDriveVoltage(driveFeedforward.calculate(velocityRadPerSec)
                + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

        return optimizedState;
    }

    public void runCharacterization(double volts) {
        io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), 0.0));
        io.setDriveVoltage(volts);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * ModuleConstants.kWheelRadiusMeters;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * ModuleConstants.kWheelRadiusMeters;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}
