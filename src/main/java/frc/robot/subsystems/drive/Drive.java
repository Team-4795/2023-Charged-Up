// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Big thanks to team 6328 Mechanical Advantage for some of the code used here.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class Drive extends SubsystemBase {
  // Create field2d
  public final Field2d m_field = new Field2d();
  
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
      
  private ChassisSpeeds setpoint = new ChassisSpeeds();
    
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
  
  private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
  
  private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
    new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
  };

  private static Drive mInstance;

    public static Drive getInstance() {
        if (mInstance == null) {
            switch (Constants.getRobot()) {
                case Comp:
                    mInstance = new Drive(
                            new GyroIONavX(),
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
  
  /** Creates a new DriveSubsystem. */
  private Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
        
    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(DriveConstants.kChassisAngularOffset),
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    });
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
      Rotation2d.fromDegrees(-getAngle() + DriveConstants.kChassisAngularOffset),
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
      
      var setpointTwist = new Pose2d().log(
        new Pose2d(
          setpoint.vxMetersPerSecond * Constants.DT,
          setpoint.vyMetersPerSecond * Constants.DT,
          new Rotation2d(setpoint.omegaRadiansPerSecond * Constants.DT)));
      
      var adjustedSpeeds = new ChassisSpeeds(
        setpointTwist.dx / Constants.DT,
        setpointTwist.dy / Constants.DT,
        setpointTwist.dtheta / Constants.DT);
      
      SwerveModuleState[] setpointStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(adjustedSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxSpeedMetersPerSecond);
      
      if (adjustedSpeeds.vxMetersPerSecond == 0.0
      && adjustedSpeeds.vyMetersPerSecond == 0.0
      && adjustedSpeeds.omegaRadiansPerSecond == 0.0) {
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
  }
    
  public double getAngle() {
    return Math.toDegrees(gyroInputs.yawPositionRad);
  }

  public double getVisionHeading(){
    double angle = (getAngle() % 360);
    if(angle > 180){
      angle -= 360;
    } else if(angle < -180){
      angle += 360;
    }

    return -angle;
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
    Rotation2d.fromDegrees(-getAngle() + DriveConstants.kChassisAngularOffset),
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

    public Command AutoStartUp(PathPlannerTrajectory traj, boolean flip) {
        return Commands.sequence(
                Commands.runOnce(() -> {
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
