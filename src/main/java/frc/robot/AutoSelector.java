package frc.robot;

import java.util.HashMap;
import java.util.List;

import javax.naming.ldap.ManageReferralControl;

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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.StateManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class AutoSelector {
    
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  private EndEffectorIntake m_intake;
  private LiftArm m_arm;
  Field2d m_field = new Field2d();
  StateManager m_manager;
  Timer time = new Timer();

  //All Path Planner paths

  //S shape path
  PathPlannerTrajectory SPath = PathPlanner.loadPath("S Path", new PathConstraints(4, 3));

  //Robot go spin while driving straight path
  PathPlannerTrajectory HolonomicDemo = PathPlanner.loadPath("Holonomic Demo", new PathConstraints(4, 3));
  //1 Game piece Auto
  PathPlannerTrajectory OneGamePiece = PathPlanner.loadPath("1 Game Piece", new PathConstraints(4, 3));
  //2 Game piece Auto
  //PathPlannerTrajectory TwoGamePiece = PathPlanner.loadPath("2 Game Piece", new PathConstraints(4, 3));

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
  public AutoSelector(DriveSubsystem drivebase, EndEffectorIntake intake, StateManager manager, LiftArm arm) {
    this.m_arm = arm;
    this.m_intake = intake;
    this.m_manager = manager;
   

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
          new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
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
          new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          drivebase::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          drivebase // Requires this drive subsystem
      ),
      new InstantCommand(() -> {
        //Put the trajectory in glass
        m_field.getObject("traj").setTrajectory(HolonomicDemo); 
      })
                  
      ));

      //Create Hashmap for 1 game peice 

      final HashMap<String, Command> AutoEventMap1 = new HashMap<>();
      //Add commands to events through the hash map
      
      AutoEventMap1.put("Score", new SequentialCommandGroup(
        
      new InstantCommand(() -> {m_manager.handleDpad(180); setStates();}),
      new RunCommand(() -> m_intake.intake(DriveConstants.kOuttakeSpeed), m_intake).withTimeout(2)

      ));//Low score cone

      AutoEventMap1.put("Intake", new SequentialCommandGroup(
        new RunCommand(() -> {m_manager.handleDpad(180); setStates();}),
        new RunCommand(() -> m_intake.intake(DriveConstants.kIntakeSpeed), m_intake).withTimeout(2)
      ));//Low Intake cone

      AutoEventMap1.put("Score2",new SequentialCommandGroup(

      new RunCommand(() -> {m_manager.handleDpad(270); setStates();}),
      new RunCommand(() -> m_intake.intake(DriveConstants.kOuttakeSpeed), m_intake).withTimeout(2)
      ));//Mid score cone
      
      //It should split the OnePath into multiple individually paths based on the stop points/events? defined in pathplanner
      //Shouldn't need a cast IDK why it doesn't work
     List<PathPlannerTrajectory> OnePathGroup =  PathPlanner.loadPathGroup("1 Game Piece", new PathConstraints(4, 3));
     
     
     
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
              AutoEventMap1),
         // new InstantCommand(drivebase::enableXstance, drivebase),
          //new WaitCommand(5),
         // new InstantCommand(drivebase::disableXstance, drivebase),
              new FollowPathWithEvents(new PPSwerveControllerCommand(
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
           AutoEventMap1),
           new InstantCommand(() -> {
            //Put the trajectory in glass
            m_field.getObject("traj").setTrajectory(PathPlanner.loadPath("1 Game Piece", new PathConstraints(4, 3))); 
          }));

          
      
      //Add One game piece auto with events 
      chooser.addOption("1 Game Piece events", OnePiecePath);



  //no event 2 game piece 
  List<PathPlannerTrajectory> OnePathGroupNoEvent =  PathPlanner.loadPathGroup("1 Game Piece", new PathConstraints(4, 3));
    
  final HashMap<String, Command> AutoEventMap1NoEvent = new HashMap<>();   
     
      //Made the path and events into one command

  Command OnePiecePathNoEvents =
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
          OnePathGroupNoEvent.get(0).getMarkers(),
          AutoEventMap1NoEvent),
     // new InstantCommand(drivebase::enableXstance, drivebase),
      //new WaitCommand(5),
     // new InstantCommand(drivebase::disableXstance, drivebase),
          new FollowPathWithEvents(new PPSwerveControllerCommand(
            OnePathGroupNoEvent.get(0),
      drivebase::getPose, // Pose supplier
    DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
      new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      drivebase::setModuleStates, // Module states consumer
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drivebase // Requires this drive subsystem
          ),
          OnePathGroupNoEvent.get(0).getMarkers(),
          AutoEventMap1NoEvent),
       new InstantCommand(() -> {
        //Put the trajectory in glass
        m_field.getObject("traj").setTrajectory(PathPlanner.loadPath("1 Game Piece", new PathConstraints(4, 3))); 
      }));

      
  
  //Add One game piece auto with no events 
  chooser.addOption("1 Game Piece", OnePiecePathNoEvents);

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
    m_field.getObject("traj").setTrajectory(exampleTrajectory); 

    })
    
    );

    
    SmartDashboard.putData("Auto Selector", chooser);
    SmartDashboard.putData(m_field);
  }

  private void setStates() {
    m_manager.getArmSetpoint().ifPresent(m_arm::setPosition);
    m_manager.getIntakeSetpoint().ifPresent(m_intake::setIntakeSpeed);
    m_manager.getWristExtended().ifPresent(m_intake::setExtendedTarget);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}