package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;



public class AutoSelector {
    
  private final SendableChooser<Command> chooser = new SendableChooser<>();


  PathPlannerTrajectory ExamplePath = PathPlanner.loadPath("ExamplePath", new PathConstraints(4, 3));

  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics));

  public AutoSelector(DriveSubsystem drivebase) {
    chooser.setDefaultOption("ExamplePath", new SequentialCommandGroup(

    new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        drivebase.resetOdometry(ExamplePath.getInitialHolonomicPose());
      }),
      new PPSwerveControllerCommand(
         ExamplePath,
          drivebase::getPose, // Pose supplier
        DriveConstants.kDriveKinematics, // SwerveDriveKinematics
          new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          drivebase::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          drivebase // Requires this drive subsystem
      )       
      ));




    chooser.addOption("exampleTrajectory", new InstantCommand(()->{
        new SwerveControllerCommand(
            
    exampleTrajectory,
    drivebase::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    new PIDController(AutoConstants.kPXController, 0, 0),
    new PIDController(AutoConstants.kPYController, 0, 0),
    new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),
    drivebase::setModuleStates,
    drivebase);

    drivebase.resetOdometry(exampleTrajectory.getInitialPose());

    }));
    SmartDashboard.putData(chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}