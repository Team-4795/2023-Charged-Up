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
import frc.robot.StateManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.Constants.OIConstants;
import frc.robot.Commands.AutoBalance;
import frc.robot.Commands.DriveCommand;

public class AutoSelector {

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final Timer timer = new Timer();
  // All Path Planner paths

  // S shape path
  PathPlannerTrajectory SPath = PathPlanner.loadPath("S Path", new PathConstraints(4, 3));

  // Robot go spin while driving straight path
  PathPlannerTrajectory HolonomicDemo = PathPlanner.loadPath("Holonomic Demo", new PathConstraints(4, 3));
  // 2 Game piece Auto part 1 (from first score to first intake)
  PathPlannerTrajectory TwoGamePiece1 = PathPlanner.loadPath("2 Game Piece 1", new PathConstraints(4, 3));
  // 2 Game piece Auto part 2 (from intake to first second score)
  PathPlannerTrajectory TwoGamePiece2 = PathPlanner.loadPath("2 Game Piece 2", new PathConstraints(4, 3));


  PathPlannerTrajectory preload_01 = PathPlanner.loadPath("Preload (01)", new PathConstraints(4, 3));
  PathPlannerTrajectory balance_10 = PathPlanner.loadPath("Balance (10)", new PathConstraints(4, 3));

  // Define Auto Selector
  public AutoSelector(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
      StateManager m_manager) {

    // Define first path option as Holonomic Demo
    chooser.setDefaultOption("Holonomic Demo", new SequentialCommandGroup(

        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(HolonomicDemo.getInitialHolonomicPose());
        }),
        new InstantCommand(() -> {
          // Put it in break mode
          drivebase.setBreakMode();
        }),
        
        new InstantCommand(() -> {
          // Put the trajectory in glass
          m_field.getObject("traj").setTrajectory(HolonomicDemo);
        }),

        new PPSwerveControllerCommand(
            HolonomicDemo,
            drivebase::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            AutoConstants.AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will only
                                           // use feedforwards.
            AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
            AutoConstants.AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving them
                                                  // 0 will only use feedforwards.
            drivebase::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            drivebase // Requires this drive subsystem
        ),
        
        new InstantCommand( () -> {
          drivebase.drive(0,0,0,true,true);
        })
    ));

    // Define another path option as S path
    chooser.addOption("S Path", new SequentialCommandGroup(

        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(SPath.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                            // right direction
        }),
        new InstantCommand(() -> {
          // Put it in break mode
          drivebase.setBreakMode();
        }),
        new InstantCommand(() -> {
          // Put the trajectory in glass
          m_field.getObject("traj").setTrajectory(SPath);
        }),
        new PPSwerveControllerCommand(
            SPath,
            drivebase::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            AutoConstants.AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will only
                                           // use feedforwards.
            AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
            AutoConstants.AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving them
                                                  // 0 will only use feedforwards.
            drivebase::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            drivebase // Requires this drive subsystem
        ),

        new InstantCommand( () -> {
          drivebase.drive(0,0,0,true,true);
        }
        )

    ));

    // Add option of two game peice split into parts
    chooser.addOption("TwoGamePieceWCommands", new SequentialCommandGroup( // REVIEW THIS BEFORE RUNNING

        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(TwoGamePiece1.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                            // right direction
        }),
        new InstantCommand(() -> {
          // Put it in break mode
          drivebase.setBreakMode();
        }),
        // move arm to intake cone
        new RunCommand(m_manager::pickCone),
        new RunCommand(() -> m_manager.handleDpad(270)),
        
        new RunCommand(() -> m_intake.intake(DriveConstants.kConeIntakeSpeed), m_intake), 
        new RunCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),

        // wait one second or so we have enough time to pick it up | Can't use intale.timeout(1) because the motor will stop spinning after
        

        new PPSwerveControllerCommand(
            TwoGamePiece1,
            drivebase::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            AutoConstants.AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will only
                                           // use feedforwards.
            AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
            AutoConstants.AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving them
                                                  // 0 will only use feedforwards.
            drivebase::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            drivebase // Requires this drive subsystem
        ),
        new InstantCommand(() -> {
          // Put the trajectory in glass
          m_field.getObject("traj").setTrajectory(TwoGamePiece1);
        }),

        // move arm to outtake cone
        new RunCommand(m_manager::pickCone),
        new RunCommand(() -> m_manager.handleDpad(180)),
        // run outake for 1 second
        new RunCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
        new RunCommand(() -> m_intake.intake(DriveConstants.kConeOuttakeSpeed), m_intake).withTimeout(1),

       // ensure it doesn't collide with scoring station and change cube to cone accordiingly

       // move arm to intake cone
       new RunCommand(m_manager::pickCone),
       new RunCommand(() -> m_manager.handleDpad(180)),
       // run outake for 1 second
       new RunCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
       new RunCommand(() -> m_intake.intake(DriveConstants.kConeIntakeSpeed), m_intake),



        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(TwoGamePiece2.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                            // right direction
        }),

        new PPSwerveControllerCommand(
            TwoGamePiece2,
            drivebase::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            AutoConstants.AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will only
                                           // use feedforwards.
            AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
            AutoConstants.AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving them
                                                  // 0 will only use feedforwards.
            drivebase::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            drivebase // Requires this drive subsystem
        ),
        new InstantCommand(() -> {
          // Put the trajectory in glass
          m_field.getObject("traj").setTrajectory(TwoGamePiece2);
        }),
        // move arm to mid cone
        new RunCommand(m_manager::pickCone),
        new RunCommand(() -> m_manager.handleDpad(270)),
        // run outake for 1 second
        new RunCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
        new RunCommand(() -> m_intake.intake(DriveConstants.kConeOuttakeSpeed), m_intake).withTimeout(1)

    ));


    chooser.addOption("Preload + balance", new SequentialCommandGroup( // REVIEW THIS BEFORE RUNNING
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        drivebase.resetOdometry(TwoGamePiece1.getInitialHolonomicPose()); // May need to rethink this so it faces the right direction
      }),
      new InstantCommand(() -> {
        // Put it in break mode
        drivebase.setBreakMode();
      }),
      // move arm to mid cone
      new RunCommand(m_manager::pickCone),
      new RunCommand(() -> m_manager.handleDpad(270)),
      // run outake for 1 second
      new RunCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
      new RunCommand(() -> m_intake.intake(DriveConstants.kConeIntakeSpeed), m_intake),

      new PPSwerveControllerCommand(
          preload_01,
          drivebase::getPose, // Pose supplier
          DriveConstants.kDriveKinematics, // SwerveDriveKinematics
          AutoConstants.AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will only
                                         // use feedforwards.
          AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
          AutoConstants.AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving them
                                                // 0 will only use feedforwards.
          drivebase::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color.
                // Optional, defaults to true
          drivebase // Requires this drive subsystem
      ),
      new InstantCommand(() -> {
        // Put the trajectory in glass
        m_field.getObject("traj").setTrajectory(preload_01);
      }),

      // move arm to outake cone
      new RunCommand(m_manager::pickCone),
      new RunCommand(() -> m_manager.handleDpad(180)),
      // run outake for 1 second
      new RunCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
      new RunCommand(() -> m_intake.intake(DriveConstants.kConeOuttakeSpeed), m_intake).withTimeout(1),

      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        drivebase.resetOdometry(balance_10.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                          // right direction
      }),

      new PPSwerveControllerCommand(
          balance_10,
          drivebase::getPose, // Pose supplier
          DriveConstants.kDriveKinematics, // SwerveDriveKinematics
          AutoConstants.AutoXcontroller, // X controller. Tune these values for your robot. Leaving them 0 will only
                                         // use feedforwards.
          AutoConstants.AutoYcontroller, // Y controller (usually the same values as X controller)
          AutoConstants.AutoRotationcontroller, // Rotation controller. Tune these values for your robot. Leaving them
                                                // 0 will only use feedforwards.
          drivebase::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color.
                // Optional, defaults to true
          drivebase // Requires this drive subsystem
      ),
      new InstantCommand(() -> {
        // Put the trajectory in glass
        m_field.getObject("traj").setTrajectory(balance_10);
      }),

      new DriveCommand(drivebase, AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold),
      new AutoBalance(drivebase, AutoConstants.angularVelocityErrorThreshold)
    ));

    chooser.addOption("Balance Only", new SequentialCommandGroup(
      new DriveCommand(drivebase, AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold),
      new AutoBalance(drivebase, AutoConstants.angularVelocityErrorThreshold)
    ));

    SmartDashboard.putData("Auto Selector", chooser);

  }
  

  public Command getSelected() {
    return chooser.getSelected();
  }
}