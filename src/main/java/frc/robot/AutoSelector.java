package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;



public class AutoSelector {
    
  private final SendableChooser<Command> chooser = new SendableChooser<>();



  //All Path Planner paths

  //S shape path
  PathPlannerTrajectory SPath = PathPlanner.loadPath("S Path", new PathConstraints(4, 3));

  //Robot go spin while driving straight path
  PathPlannerTrajectory HolonomicDemo = PathPlanner.loadPath("Holonomic Demo", new PathConstraints(4, 3));
  //1 Game piece Auto
  PathPlannerTrajectory OneGamePiece = PathPlanner.loadPath("1 Game Piece", new PathConstraints(4, 3));
  //2 Game piece Auto


  //Manually constructed path S shape
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






  //Define Auto Selector
  public AutoSelector(DriveSubsystem drivebase) {

    final HashMap<String, Command> AutoEventMap = new HashMap<>();

    //Define first path option as S path
    chooser.setDefaultOption("SPath", new SequentialCommandGroup(

    new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        drivebase.resetOdometry(SPath.getInitialHolonomicPose()); //May need to rethink this so it faces the right direction
      }),
      new PPSwerveControllerCommand(
         SPath,
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

       //Define another path option as Holonomic Demo path
    chooser.addOption("Holonomic Demo", new SequentialCommandGroup(

    new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        drivebase.resetOdometry(HolonomicDemo.getInitialHolonomicPose()); //May need to rethink this so it faces the right direction
      }),
      new PPSwerveControllerCommand(
        HolonomicDemo,
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

      //Add commands to events through the hash map\
      //Add the actual commands instead of the print commands
      AutoEventMap.put("event1", new PrintCommand("event 1 here"));
      AutoEventMap.put("event2", new PrintCommand("event 2 here"));

      
      //It should split the OnePath into multiple individually paths based on the stop points/events? defined in pathplanner
      //Shouldn't need a cast IDK why it doesn't work
      ArrayList<PathPlannerTrajectory> OnePathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("1 Game Piece", new PathConstraints(4, 3));
     
     
     
          //Made the path and events into one command

      Command OnePiecePath =
          new SequentialCommandGroup( 
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              drivebase.resetOdometry(OnePathGroup.get(0).getInitialHolonomicPose()); //May need to rethink this so it faces the right direction
            }),
            new FollowPathWithEvents(
              new PPSwerveControllerCommand(
                
              OnePathGroup.get(0),
          drivebase::getPose, // Pose supplier
        DriveConstants.kDriveKinematics, // SwerveDriveKinematics
          new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          drivebase::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          drivebase // Requires this drive subsystem
              ),
              OnePathGroup.get(0).getMarkers(),
              AutoEventMap),
         // new InstantCommand(drivebase::enableXstance, drivebase),
          //new WaitCommand(5),
         // new InstantCommand(drivebase::disableXstance, drivebase),
              new FollowPathWithEvents(new PPSwerveControllerCommand(
                OnePathGroup.get(1),
          drivebase::getPose, // Pose supplier
        DriveConstants.kDriveKinematics, // SwerveDriveKinematics
          new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          drivebase::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          drivebase // Requires this drive subsystem
              ),
              OnePathGroup.get(1).getMarkers(),
           AutoEventMap));

          
      
      //Add One game piece auto with events 
      chooser.addOption("1 Game Piece", OnePiecePath);




      //Add another option which is the manually made S path
    chooser.addOption("Manual SPath", new InstantCommand(()->{
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