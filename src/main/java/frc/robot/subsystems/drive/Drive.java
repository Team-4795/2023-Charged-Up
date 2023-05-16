// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.intake.Intake;
import frc.utils.RotationMatrix;
import frc.utils.VisionPoseEstimator;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    // Create field2d
    public final Field2d m_field = new Field2d();

    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    // The gyro sensor

    // AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private static Drive mInstance;

    public static Drive getInstance() {
        if (mInstance == null) {
            switch (Constants.getRobot()) {
                case Comp:
                    mInstance = new Drive(
                            new GyroIOSim(),
                            new ModuleIOSparkMax(0),
                            new ModuleIOSparkMax(1),
                            new ModuleIOSparkMax(2),
                            new ModuleIOSparkMax(3));
                    break;
                case Sim:
                    mInstance = new Drive(
                            new GyroIOSim(),
                            new ModuleIOSim(),
                            new ModuleIOSim(),
                            new ModuleIOSim(),
                            new ModuleIOSim());
                    break;
            }
        }

        return mInstance;
    }

    private static final double coastThresholdMetersPerSec =
            0.05; // Need to be under this to switch to coast when disabling
    private static final double coastThresholdSecs =
            6.0; // Need to be under the above speed for this length of time to switch to coast

    private Timer lastMovementTimer = new Timer();

    private RotationMatrix rotation;
    private double balanceSpeed = 0.0;

    private ChassisSpeeds setpoint = new ChassisSpeeds();

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry;

    private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};

    private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };

    VisionPoseEstimator poseEstimator = new VisionPoseEstimator(VisionConstants.kSnakeEyesCamera);
    Pose2d visionPose = new Pose2d();
    /** Creates a new DriveSubsystem. */
    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        SmartDashboard.putData(m_field);

        m_odometry = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(Constants.DriveConstants.kChassisAngularOffset),
                new SwerveModulePosition[] {
                    modules[0].getPosition(),
                    modules[1].getPosition(),
                    modules[2].getPosition(),
                    modules[3].getPosition()
                });

        setDefaultCommand(new DriveWithJoysticks(
                this,
                () -> -MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(-OIConstants.driverController.getLeftX(), OIConstants.kDriveDeadband),
                () -> MathUtil.applyDeadband(-OIConstants.driverController.getRightX(), OIConstants.kDriveDeadband),
                () -> false));
    }

    public static Twist2d log(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < 1E-9) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta =
                    -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
        }
        final Translation2d translation_part =
                transform.getTranslation().rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }

    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
        Logger.getInstance().recordOutput("Gyro angle", -gyroInputs.yawPositionRad);

        m_odometry.update(
                Rotation2d.fromDegrees(-getAngle() + Constants.DriveConstants.kChassisAngularOffset),
                new SwerveModulePosition[] {
                    modules[0].getPosition(),
                    modules[1].getPosition(),
                    modules[2].getPosition(),
                    modules[3].getPosition()
                });

        for (var module : modules) {
            module.periodic();
        }

        // Run modules
        if (DriverStation.isDisabled()) {
            // Stop moving while disabled
            for (var module : modules) {
                module.stop();
            }

            // Clear setpoint logs
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

        } else {
            // Calculate module setpoints

            var setpointTwist = new Pose2d()
                    .log(new Pose2d(
                            setpoint.vxMetersPerSecond * Constants.loopPeriodSecs,
                            setpoint.vyMetersPerSecond * Constants.loopPeriodSecs,
                            new Rotation2d(setpoint.omegaRadiansPerSecond * Constants.loopPeriodSecs)));

            var adjustedSpeeds = new ChassisSpeeds(
                    setpointTwist.dx / Constants.loopPeriodSecs,
                    setpointTwist.dy / Constants.loopPeriodSecs,
                    setpointTwist.dtheta / Constants.loopPeriodSecs);

            SwerveModuleState[] setpointStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(adjustedSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxSpeedMetersPerSecond);

            if (adjustedSpeeds.vxMetersPerSecond == 0.0
                    && adjustedSpeeds.vyMetersPerSecond == 0.0
                    && adjustedSpeeds.omegaRadiansPerSecond == 0) {
                for (int i = 0; i < 4; i++) {
                    setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
                }
            }
            lastSetpointStates = setpointStates;

            // Send setpoints to modules
            SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
            }

            // Log setpoint states
            Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
            Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
        }

        // Log measured states
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = modules[i].getState();
        }
        Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);

        // Update odometry
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (modules[i].getPositionMeters() - lastModulePositionsMeters[i]), modules[i].getAngle());
            lastModulePositionsMeters[i] = modules[i].getPositionMeters();
        }
        var twist = DriveConstants.kDriveKinematics.toTwist2d(wheelDeltas);
        gyroIO.addOffset(-twist.dtheta);

        // var gyroYaw = new Rotation2d(gyroInputs.yawPositionRad);
        // if (gyroInputs.connected) {
        //   twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
        // }
        Logger.getInstance().recordOutput("Odometry/Robot", getPose());

        // Update brake mode
        boolean stillMoving = false;
        for (int i = 0; i < 4; i++) {
            if (Math.abs(modules[i].getVelocityMetersPerSec()) > coastThresholdMetersPerSec) {
                stillMoving = true;
            }
        }
        if (stillMoving) lastMovementTimer.reset();
    }

    public double getAngle() {
        return Math.toDegrees(gyroInputs.yawPositionRad);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        setpoint = speeds;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        runVelocity(DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates));
    }

    public double getElevationAngle() {
        return Math.toDegrees(gyroInputs.pitchPositionRad);
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.kMaxSpeedMetersPerSecond;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return DriveConstants.kMaxAngularSpeed;
    }

    /** Returns the current odometry pose. */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return m_odometry.getPoseMeters().getRotation();
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (var module : modules) {
            driveVelocityAverage += module.getCharacterizationVelocity();
        }
        return driveVelocityAverage / 4.0;
    }

    public void setBrakeMode() {
        for (var module : modules) {
            module.setBrakeMode(true);
        }
    }

    public void zeroHeading() {
        gyroIO.reset();
        gyroIO.setOffset(0);
    }

    public void zeroReverseHeading() {
        gyroIO.reset();
        gyroIO.setOffset(180);
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(-getAngle() + Constants.DriveConstants.kChassisAngularOffset),
                new SwerveModulePosition[] {
                    modules[0].getPosition(),
                    modules[1].getPosition(),
                    modules[2].getPosition(),
                    modules[3].getPosition()
                },
                pose);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
        return new PPSwerveControllerCommand(
                traj,
                this::getPose, // Pose supplier
                DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                AutoConstants
                        .AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will only
                // use feedforwards.
                AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
                AutoConstants
                        .AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving them
                // 0 will only use feedforwards.
                this::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color.
                // Optional, defaults to true
                this // Requires this drive subsystem
                );
    }

    public Command AutoStartUp(PathPlannerTrajectory traj, boolean flip, Intake m_intake) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (flip) {
                        zeroReverseHeading();
                    } else {
                        zeroHeading();
                    }

                    resetOdometry(
                            PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance())
                                    .getInitialHolonomicPose());

                    setBrakeMode();
                })
                // new InstantCommand(() -> m_intake.setOverrideStoring(true))
                );
    }
}
