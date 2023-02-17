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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.StateManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.Constants.OIConstants;

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
  //PathPlannerTrajectory TwoGamePiece = PathPlanner.loadPath("2 Game Piece", new PathConstraints(4, 3));


  //Define Auto Selector
  public AutoSelector(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field ,  StateManager m_manager) {

   

   
    //Define first path option as S path
    chooser.setDefaultOption("SPath", new SequentialCommandGroup(

    new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
       // drivebase.resetOdometry(ExamplePath.getInitialHolonomicPose());
      }),
      new InstantCommand(() -> {
        //Put it in break mode
        drivebase.setBreakMode();
      }),
      new PPSwerveControllerCommand(
         SPath,
          drivebase::getPose, // Pose supplier
        DriveConstants.kDriveKinematics, // SwerveDriveKinematics 
        AutoConstants.AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
        AutoConstants.AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          drivebase::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          drivebase // Requires this drive subsystem
      ),
      new InstantCommand(() -> {
        //Put the trajectory in glass
        m_field.getObject("traj").setTrajectory(SPath); 
      })
            
      ));

       //Define another path option as Holonomic Demo path
    chooser.addOption("Holonomic Demo", new SequentialCommandGroup(

    new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        drivebase.resetOdometry(HolonomicDemo.getInitialHolonomicPose()); //May need to rethink this so it faces the right direction
      }),
      new InstantCommand(() -> {
        //Put it in break mode
        drivebase.setBreakMode();
      }),
      new PPSwerveControllerCommand(
        HolonomicDemo,
          drivebase::getPose, // Pose supplier
        DriveConstants.kDriveKinematics, // SwerveDriveKinematics
        AutoConstants.AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
        AutoConstants.AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          drivebase::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          drivebase // Requires this drive subsystem
      ),
      new InstantCommand(() -> {
        //Put the trajectory in glass
        m_field.getObject("traj").setTrajectory(HolonomicDemo); 
      })
                  
      ));

    
    SmartDashboard.putData("Auto Selector", chooser);

  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}