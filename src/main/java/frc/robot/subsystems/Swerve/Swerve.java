package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.utils.SwerveUtils;
import frc.robot.subsystems.vision.Vision;

public class Swerve extends SubsystemBase {
    // Create field2d
    public final Field2d m_field = new Field2d();

    // Create MAXSwerveModules

    private final SwerveModule m_frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule m_frontRight = new SwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule m_rearLeft = new SwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final SwerveModule m_rearRight = new SwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor

    // AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // PoseEstimater

    // private SwerveDrivePoseEstimator m_poseEstimator;

    WPI_Pigeon2 m_gyro = new WPI_Pigeon2(20);

    private double balanceSpeed = 0.0;
    public int oscillations = 0;

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;


    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(-m_gyro.getAngle() + Constants.DriveConstants.kChassisAngularOffset),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    SwerveDrivePoseEstimator m_poseEstimator =
            new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(-m_gyro.getAngle() + Constants.DriveConstants.kChassisAngularOffset),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition(),
            
                }, new Pose2d(0,0, new Rotation2d()));
                

    Pose2d visionPose = new Pose2d();

    private static Swerve instance;

    public static Swerve getInstance(){
        if(instance == null){
            instance = new Swerve();
        }
        return instance;
    }

    /** Creates a new DriveSubsystem. */
    private Swerve() {
        SmartDashboard.putData(m_field);
        setDefaultCommand(
            run(() -> this.drive(
            MathUtil.applyDeadband(OIConstants.driverController.getRawAxis(0), OIConstants.kDriveDeadband),
            MathUtil.applyDeadband(-OIConstants.driverController.getRawAxis(1), OIConstants.kDriveDeadband),
            MathUtil.applyDeadband(-OIConstants.driverController.getRawAxis(4), OIConstants.kDriveDeadband),
            true,true))
        );
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                Rotation2d.fromDegrees(-m_gyro.getAngle() + Constants.DriveConstants.kChassisAngularOffset),

                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
        m_poseEstimator.update(
            Rotation2d.fromDegrees(-m_gyro.getAngle() + Constants.DriveConstants.kChassisAngularOffset),
    
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        });
        m_field.setRobotPose(m_odometry.getPoseMeters());

        Pose3d visionMeasurement3d = new Pose3d(Vision.getInstance().loggedpose[0], Vision.getInstance().loggedpose[1], Vision.getInstance().loggedpose[2], Vision.getInstance().botRotation);
        Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();
        double distance = visionMeasurement2d.getTranslation().getDistance(m_poseEstimator.getEstimatedPosition().getTranslation());

        boolean forceApriltags = Constants.OIConstants.operatorController.getHID().getStartButton();
        double angleDiff = visionMeasurement2d.getRotation().getRadians() - m_poseEstimator.getEstimatedPosition().getRotation().getRadians();

        if (visionMeasurement2d != null && visionMeasurement2d.getX() != 0 && (forceApriltags || (distance < 1.0 && Math.abs(angleDiff) < 1.0))) {
            var stddevs = VecBuilder.fill(1.5, 1.5, 1);

            if (Vision.getInstance().getDistance() > 2.0) {
                stddevs = VecBuilder.fill(3, 3, 2);
            }

            m_poseEstimator.addVisionMeasurement(visionMeasurement2d, Timer.getFPGATimestamp() - (Vision.getInstance().botpose[6] / 1000.0), stddevs);
        }

        SmartDashboard.putNumber("rotation", getPose().getRotation().getDegrees());
        Logger.getInstance().recordOutput("Gyro/Yaw",  m_gyro.getAngle());
        Logger.getInstance().recordOutput("Gyro/Pitch",  m_gyro.getPitch());
        Logger.getInstance().recordOutput("Gyro/Roll",  m_gyro.getRoll());

        SmartDashboard.putData("pose", m_field);
        Logger.getInstance().recordOutput("Estimated pose", m_poseEstimator.getEstimatedPosition());

        SmartDashboard.putNumber("Pigeon Angle of Elevation", getElevationAngle());
        SmartDashboard.putNumber("NavX Angle of Elevation", m_gyro.getPitch());
        SmartDashboard.putNumber("Balancing Speed", getBalanceSpeed());
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putNumber("backwards", Math.cos(Math.toRadians(this.getAngle())));
        SmartDashboard.putNumberArray("Swerve states", getModuleStates());
        // SmartDashboard.putNumber("magnetometer X", m_gyro.getRawMagX());
        // SmartDashboard.putNumber("magnetometer Y", m_gyro.getRawMagY());
        // SmartDashboard.putNumber("magnetometer Z", m_gyro.getRawMagZ());
        SmartDashboard.putNumberArray("Odometry",
                new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });
        // SmartDashboard.putNumberArray("Vision Pose Estimate", new double[] {
        // visionPose.getX(), visionPose.getY(), visionPose.getRotation().getDegrees()
        // });
    }

    public void resetEstimatedPose() {
        m_poseEstimator.resetPosition(
                Rotation2d.fromDegrees(-m_gyro.getAngle() + Constants.DriveConstants.kChassisAngularOffset),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                new Pose2d());
    }
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose, boolean flip) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(-m_gyro.getAngle() + Constants.DriveConstants.kChassisAngularOffset),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }
        

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
        
        SmartDashboard.putNumber("xSpeed", xSpeedDelivered);
        SmartDashboard.putNumber("ySpeed", ySpeedDelivered);
        SmartDashboard.putNumber("rotSpeed", rotDelivered);
        

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(
                                        -m_gyro.getAngle() + Constants.DriveConstants.kChassisAngularOffset))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void runVelocity(ChassisSpeeds speeds){
        this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true, true);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    public void setBalanceSpeed(double value) {
        balanceSpeed = value;
    }

    public void setOscillations(int oscillations) {
        this.oscillations = oscillations;
    }

    // pick up these changes please
    public double getBalanceSpeed() {
        return balanceSpeed;
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
        // m_gyro.setAngleAdjustment(0);
    }

    public void zeroReverseHeading() {
        m_gyro.reset();
        // m_gyro.setAngleAdjustment(180);
        m_gyro.addYaw(180);

    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getAngle() + Constants.DriveConstants.kChassisAngularOffset);
    }


    public double getElevationAngle() {
        // return pigeon.getPitch();
        return m_gyro.getPitch();
    }


    public double getVisionHeading() {
        double angle = (getAngle()) % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }

        // angle = ((angle - 180) % 360) + 180;

        return -angle;
    }

    public double[] getModuleStates() {
        double[] swerveStates = new double[8];
        swerveStates[0] = m_frontLeft.getState().angle.getDegrees();
        swerveStates[1] = m_frontLeft.getState().speedMetersPerSecond;
        swerveStates[2] = m_frontRight.getState().angle.getDegrees();
        swerveStates[3] = m_frontRight.getState().speedMetersPerSecond;
        swerveStates[4] = m_rearLeft.getState().angle.getDegrees();
        swerveStates[5] = m_rearLeft.getState().speedMetersPerSecond;
        swerveStates[6] = m_rearRight.getState().angle.getDegrees();
        swerveStates[7] = m_rearRight.getState().speedMetersPerSecond;
        return swerveStates;
    }

    public void setBreakMode() {
        m_frontLeft.setBreakMode();
        m_frontRight.setBreakMode();
        m_rearLeft.setBreakMode();
        m_rearRight.setBreakMode();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
        return new PPSwerveControllerCommand(
                traj,
                this::getPose, // Pose supplier
                DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                AutoConstants.AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will
                                               // only
                                               // use feedforwards.
                AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
                AutoConstants.AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving
                                                      // them
                                                      // 0 will only use feedforwards.
                this::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color.
                      // Optional, defaults to true
                this // Requires this drive subsystem
        );
    }

    public Command AutoStartUp(PathPlannerTrajectory traj, boolean flip) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (flip) {
                        zeroReverseHeading();
                    } else {
                        zeroHeading();
                    }}),
                new WaitCommand(0.2),
                new InstantCommand(() -> {

                    this.resetOdometry(PathPlannerTrajectory
                            .transformTrajectoryForAlliance(traj, DriverStation.getAlliance())
                            .getInitialHolonomicPose(), flip);

                    this.setBreakMode();
                }));
        // new InstantCommand(() -> m_intake.setOverrideStoring(true))
    }

    public double getAngle() {
        return m_gyro.getAngle();
    }

    public void setCoastMode() {
        m_frontLeft.setCoastMode();
        m_frontRight.setCoastMode();
        m_rearLeft.setCoastMode();
        m_rearRight.setCoastMode();
    }
}
