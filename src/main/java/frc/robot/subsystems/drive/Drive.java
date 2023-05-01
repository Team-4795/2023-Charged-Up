// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.MAXSwerveModule;
import frc.utils.RotationMatrix;
import frc.utils.SwerveUtils;
import frc.utils.VisionPoseEstimator;

public class Drive extends SubsystemBase {
  // Create field2d
  public final Field2d m_field = new Field2d();

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  // The gyro sensor

  // AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private RotationMatrix rotation;
  private double balanceSpeed = 0.0;

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

  VisionPoseEstimator poseEstimator = new VisionPoseEstimator(VisionConstants.kSnakeEyesCamera);
  Pose2d visionPose = new Pose2d();
  /** Creates a new DriveSubsystem. */
  public Drive(
    GyroIO gyroIO,
    ModuleIO flModuleIO,
    ModuleIO frModuleIO,
    ModuleIO blModuleIO,
    ModuleIO brModuleIO
  ) {
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
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    // SmartDashboard.putNumber("AngleYaw", m_gyro.getYaw());
    // SmartDashboard.putNumber("Angle", m_gyro.getAngle());
    // SmartDashboard.putBoolean("isconnected", m_gyro.isConnected());
    // SmartDashboard.putBoolean("iscalibrating", m_gyro.isCalibrating());

    gyroIO.updateInputs(gyroInputs);

    for (var module: modules) {
      module.periodic();
    }

    m_odometry.update(
        // Rotation2d.fromDegrees(-m_gyro.getAngle() + Constants.DriveConstants.kChassisAngularOffset),
        Rotation2d.fromDegrees(-getAngle() + Constants.DriveConstants.kChassisAngularOffset),

        new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
        });

    m_field.setRobotPose(m_odometry.getPoseMeters());
    poseEstimator.estimateRobotPose(this.getPose());
    visionPose = poseEstimator.getPoseEstimate();    

    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition(
              (modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
              modules[i].getAngle());
      lastModulePositionsMeters[i] = modules[i].getPositionMeters();
    }
    var twist = DriveConstants.kDriveKinematics.toTwist2d(wheelDeltas);
    gyroIO.addOffset(-twist.dtheta);

    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
      optimizedStates[i] = modules[i].setpoints;
    }
    Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);
    Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", measuredStates);


    
    
    SmartDashboard.putNumber("rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Gyro angle", gyroInputs.yawPositionRad);
    SmartDashboard.putData("pose", m_field);

    SmartDashboard.putNumber("Angle of Elevation", getElevationAngle());
    SmartDashboard.putNumber("Roll", Math.toDegrees(gyroInputs.rollPositionRad));
    SmartDashboard.putNumber("Balancing Speed", getBalanceSpeed());
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("backwards", Math.cos(Math.toRadians(this.getAngle())));
    SmartDashboard.putNumber("Vision heading", getvisionheading());
    SmartDashboard.putNumberArray("Swerve states", getModuleStates());

    Logger.getInstance().recordOutput("Odometry", getPose());
    //SmartDashboard.putNumberArray("Vision Pose Estimate", new double[] {
     // visionPose.getX(), visionPose.getY(), visionPose.getRotation().getDegrees() });
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
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
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

    SmartDashboard.putNumber("X speed", xSpeedDelivered);
    SmartDashboard.putNumber("Y speed", ySpeedDelivered);


    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-getAngle() + Constants.DriveConstants.kChassisAngularOffset))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  //   modules[0].resetEncoders();
  //   modules[1].resetEncoders();
  //   modules[2].resetEncoders();
  //   modules[3].resetEncoders();
  // }

  public void setBalanceSpeed(double value) {
    balanceSpeed = value;
  }

  // pick up these changes please
  public double getBalanceSpeed() {
    return balanceSpeed;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyroIO.reset();
    gyroIO.setOffset(0);
    gyroIO.updateInputs(gyroInputs);
  }

  public void zeroReverseHeading() {
    gyroIO.reset();
    gyroIO.setOffset(180);
    gyroIO.updateInputs(gyroInputs);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-getAngle() + Constants.DriveConstants.kChassisAngularOffset);
  } 

  // angle between xy-plane and the forward vector of the drivebase - potentially
  // doesn't work
  public double getElevationAngle() {
    return Math.toDegrees(gyroInputs.pitchPositionRad);
  }

  public double getElevationVelocityV2() {
    return 0.0; // in the process of remaking
  }

  public double getvisionheading() {
    double angle = (getAngle()) % 360;
    if (angle > 180) {
      angle -= 360;
    } else if (angle < -180) {
      angle += 360;
    }

    //angle = ((angle - 180) % 360) + 180;

    return -angle;
  }

  public double[] getModuleStates() {
    double[] swerveStates = new double[8];
    swerveStates[0] = modules[0].getState().angle.getRadians();
    swerveStates[1] = modules[0].getState().speedMetersPerSecond;
    swerveStates[2] = modules[1].getState().angle.getRadians();
    swerveStates[3] = modules[1].getState().speedMetersPerSecond;
    swerveStates[4] = modules[2].getState().angle.getRadians();
    swerveStates[5] = modules[2].getState().speedMetersPerSecond;
    swerveStates[6] = modules[3].getState().angle.getRadians();
    swerveStates[7] = modules[3].getState().speedMetersPerSecond;
    return swerveStates;
  }

  // public void setBreakMode() {
  //   modules[0].setBreakMode();
  //   modules[1].setBreakMode();
  //   modules[2].setBreakMode();
  //   modules[3].setBreakMode();
  // }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  //   return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
      return new PPSwerveControllerCommand(
          traj,
          this::getPose, // Pose supplier
          DriveConstants.kDriveKinematics, // SwerveDriveKinematics
          AutoConstants.AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will only
                                          // use feedforwards.
          AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
          AutoConstants.AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving them
                                                // 0 will only use feedforwards.
          this::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color.
                // Optional, defaults to true
          this // Requires this drive subsystem
      );
  }

  public Command AutoStartUp(PathPlannerTrajectory traj, boolean flip, EndEffectorIntake m_intake) {
    return 
        new SequentialCommandGroup( 
          new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (flip) {
            zeroReverseHeading();
          } else {
            zeroHeading();
          }

          var pose = PathPlannerTrajectory
            .transformTrajectoryForAlliance(traj, DriverStation.getAlliance())
            .getInitialHolonomicPose();

          // pose = new Pose2d(pose.getTranslation(), new Rotation2d());
          
          this.resetOdometry(pose);
          
          // this.setBreakMode();
        })
         // new InstantCommand(() -> m_intake.setOverrideStoring(true))
          );

  }

  public double getAngle() {
    return Math.toDegrees(gyroInputs.yawPositionRad);
  }
}
