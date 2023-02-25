package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.TapeAlign;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;


public class AutoSelector {

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final Timer timer = new Timer();
  // All Path Planner paths

  // S shape path
  PathPlannerTrajectory SPath = PathPlanner.loadPath("S Path", new PathConstraints(1, 1));

  // Robot go spin while driving straight path
  PathPlannerTrajectory HolonomicDemo = PathPlanner.loadPath("Holonomic Demo", new PathConstraints(1, 1));
  // Cone 2 Game piece Auto part 1 (from first score to first intake)
  PathPlannerTrajectory ConeTwoGamePiece1 = PathPlanner.loadPath("Cone 2 Game Piece 1", new PathConstraints(1, 1));
  // Cone 2 Game piece Auto part 2 (from intake to first second score)
  PathPlannerTrajectory ConeTwoGamePiece2 = PathPlanner.loadPath("Cone 2 Game Piece 2", new PathConstraints(1, 1));

 //Cube 2 Game piece Auto part 1 (from first score to first intake)
 PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Cube 2 Game Piece 1", new PathConstraints(1, 1));
 //Cube  2 Game piece Auto part 2 (from intake to first second score)
 PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Cube 2 Game Piece 2", new PathConstraints(1, 1));


//Auto Balence with 1 cone
PathPlannerTrajectory AutoBalence = PathPlanner.loadPath("Auto Balence", new PathConstraints(1, 1));

  // Define Auto Selector
  public AutoSelector(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field, StateManager m_manager, Vision m_vision) {

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
    chooser.addOption("TwoGamePiece", new SequentialCommandGroup( // REVIEW THIS BEFORE RUNNING

        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(ConeTwoGamePiece1.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                            // right direction
        }),
        new InstantCommand(() -> {
          // Put it in break mode
          drivebase.setBreakMode();
        }),
        new InstantCommand(() -> {
          // Put the trajectory in glass
          m_field.getObject("traj").setTrajectory(ConeTwoGamePiece1);
        }),

        new PPSwerveControllerCommand(
          ConeTwoGamePiece1,
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
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(ConeTwoGamePiece2.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                            // right direction
        }),
        new InstantCommand(() -> {
          // Put the trajectory in glass
          m_field.getObject("traj").setTrajectory(ConeTwoGamePiece2);
        }),

        new PPSwerveControllerCommand(
          ConeTwoGamePiece2,
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



    // Add option of Vision based two game peice split into parts with commands
    chooser.addOption("ConeVisionTwoGamePieceWCommands", new SequentialCommandGroup( 
        
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(ConeTwoGamePiece1.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                            // right direction
        }),
        new InstantCommand(() -> {
          // Put it in break mode
          drivebase.setBreakMode();
        }),
        //Set pipeline to tape 
        new InstantCommand(() -> {
          m_vision.switchToTape();
        }),
        //Align
        new TapeAlign(
          drivebase,
          m_vision,() -> AutoConstants.VisionXspeed, () ->AutoConstants.VisionYspeed ).withTimeout(1.5),
  
        // move arm to intake score mid
        new InstantCommand(m_manager::pickCone),
        new InstantCommand(() -> m_manager.handleDpad(270)),
        new InstantCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
        new InstantCommand(() ->  m_manager.getOuttakeSetpoint().ifPresent(m_intake::setOuttakeSpeed)),
        new InstantCommand(() -> m_manager.getWristExtended().ifPresent(m_intake::setExtendedTarget)),
        new WaitUntilCommand(m_arm::atSetpoint),
        // outake in order to score pre loaded
        // Use RunCommand to continuously run this
        new RunCommand(m_intake::outtake, m_intake).withTimeout(1),
        new WaitCommand(1),
        
        new InstantCommand(() -> {
          // Put the trajectory in glass
          m_field.getObject("traj").setTrajectory(ConeTwoGamePiece1);
        }),

        new PPSwerveControllerCommand(
          ConeTwoGamePiece1,
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
        
       // move arm to intake low cone
       new InstantCommand(m_manager::pickCone),
       new InstantCommand(() -> m_manager.handleDpad(180)),
       new InstantCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
       new InstantCommand(() ->  m_manager.getOuttakeSetpoint().ifPresent(m_intake::setOuttakeSpeed)),
       new InstantCommand(() -> m_manager.getWristExtended().ifPresent(m_intake::setExtendedTarget)),
       new WaitUntilCommand(m_arm::atSetpoint),
        // Run intake for 1 seconds
       new WaitCommand(1),
        //Do I need an intake command?
      


        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(ConeTwoGamePiece2.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                            // right direction
        }),
        new InstantCommand(() -> {
          // Put the trajectory in glass
          m_field.getObject("traj").setTrajectory(ConeTwoGamePiece2);
        }),

        new PPSwerveControllerCommand(
          ConeTwoGamePiece2,
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
        //Align
        new TapeAlign(
          drivebase,
          m_vision,() -> AutoConstants.VisionXspeed, () ->AutoConstants.VisionYspeed ).withTimeout(1.5),
  
        // move arm to low cone
        new InstantCommand(m_manager::pickCone),
        new InstantCommand(() -> m_manager.handleDpad(180)),
        new InstantCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
        new InstantCommand(() ->  m_manager.getOuttakeSetpoint().ifPresent(m_intake::setOuttakeSpeed)),
        new InstantCommand(() -> m_manager.getWristExtended().ifPresent(m_intake::setExtendedTarget)),
        new WaitUntilCommand(m_arm::atSetpoint),
        //Run outake for 1 second to score
        new RunCommand(m_intake::outtake, m_intake).withTimeout(1)

    ));

  
  
  
  
    // Add option of  Vision based two game peice split into parts with commands Cube
    chooser.addOption("CubeVisionTwoGamePieceWCommands", new SequentialCommandGroup( 

            
    new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      drivebase.resetOdometry(CubeTwoGamePiece1.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                        // right direction
    }),
    new InstantCommand(() -> {
      // Put it in break mode
      drivebase.setBreakMode();
    }),
    //Set pipeline to tag 
    new InstantCommand(() -> {
      m_vision.switchToTag();
    }),
    //Align
    new TapeAlign(
      drivebase,
      m_vision,() -> AutoConstants.VisionXspeed, () ->AutoConstants.VisionYspeed ).withTimeout(1.5),

    // move arm to intake score mid
    new InstantCommand(m_manager::pickCube),
    new InstantCommand(() -> m_manager.handleDpad(270)),
    new InstantCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
    new InstantCommand(() ->  m_manager.getOuttakeSetpoint().ifPresent(m_intake::setOuttakeSpeed)),
    new InstantCommand(() -> m_manager.getWristExtended().ifPresent(m_intake::setExtendedTarget)),
    new WaitUntilCommand(m_arm::atSetpoint),
    // outake in order to score pre loaded
    // Use RunCommand to continuously run this
    new RunCommand(m_intake::outtake, m_intake).withTimeout(1),
    new WaitCommand(1),
    
    new InstantCommand(() -> {
      // Put the trajectory in glass
      m_field.getObject("traj").setTrajectory(CubeTwoGamePiece1);
    }),

    new PPSwerveControllerCommand(
      CubeTwoGamePiece1,
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
    
   // move arm to intake low cone
   new InstantCommand(m_manager::pickCube),
   new InstantCommand(() -> m_manager.handleDpad(180)),
   new InstantCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
   new InstantCommand(() ->  m_manager.getOuttakeSetpoint().ifPresent(m_intake::setOuttakeSpeed)),
   new InstantCommand(() -> m_manager.getWristExtended().ifPresent(m_intake::setExtendedTarget)),
   new WaitUntilCommand(m_arm::atSetpoint),
    // Run intake for 1 seconds
   new WaitCommand(1),
    //Do I need an intake command?
  


    new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      drivebase.resetOdometry(CubeTwoGamePiece2.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                        // right direction
    }),
    new InstantCommand(() -> {
      // Put the trajectory in glass
      m_field.getObject("traj").setTrajectory(CubeTwoGamePiece2);
    }),

    new PPSwerveControllerCommand(
      CubeTwoGamePiece2,
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
    //Align
    new TapeAlign(
      drivebase,
      m_vision,() -> AutoConstants.VisionXspeed, () ->AutoConstants.VisionYspeed ).withTimeout(1.5),

    // move arm to low cube
    new InstantCommand(m_manager::pickCube),
    new InstantCommand(() -> m_manager.handleDpad(180)),
    new InstantCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
    new InstantCommand(() ->  m_manager.getOuttakeSetpoint().ifPresent(m_intake::setOuttakeSpeed)),
    new InstantCommand(() -> m_manager.getWristExtended().ifPresent(m_intake::setExtendedTarget)),
    new WaitUntilCommand(m_arm::atSetpoint),
    //Run outake for 1 second to score
    new RunCommand(m_intake::outtake, m_intake).withTimeout(1)
    ));





    //Auto Balence with 1 cone 
    
    chooser.addOption("Auto Balance", new SequentialCommandGroup( 

            
    new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      drivebase.resetOdometry(AutoBalence.getInitialHolonomicPose()); // May need to rethink this so it faces the
                                                                        // right direction
    }),
    new InstantCommand(() -> {
      // Put it in break mode
      drivebase.setBreakMode();
    }),
    //Set pipeline to tape 
    new InstantCommand(() -> {
      m_vision.switchToTape();
    }),
    //Align
    new TapeAlign(
      drivebase,
      m_vision,() -> AutoConstants.VisionXspeed, () ->AutoConstants.VisionYspeed ).withTimeout(1.5),

    // move arm to intake score mid
    new InstantCommand(m_manager::pickCone),
    new InstantCommand(() -> m_manager.handleDpad(270)),
    new InstantCommand(() ->  m_manager.getArmSetpoint().ifPresent(m_arm::setPosition)),
    new InstantCommand(() ->  m_manager.getOuttakeSetpoint().ifPresent(m_intake::setOuttakeSpeed)),
    new InstantCommand(() -> m_manager.getWristExtended().ifPresent(m_intake::setExtendedTarget)),
    new WaitUntilCommand(m_arm::atSetpoint),
    // outake in order to score pre loaded
    new RunCommand(m_intake::outtake, m_intake).withTimeout(1),
    new WaitCommand(1),

    new PPSwerveControllerCommand(
      AutoBalence,
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

   new DriveCommand(drivebase, -AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold, AutoConstants.checkDuration),
  //new RunCommand(()-> m_robotDrive.drive(0, 0, -(Math.PI / 4), false, false), m_robotDrive).withTimeout(1),
  new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold)
    
    ));

    

    SmartDashboard.putData("Auto Selector", chooser);

  }
  

  public Command getSelected() {
    return chooser.getSelected();
  }
}