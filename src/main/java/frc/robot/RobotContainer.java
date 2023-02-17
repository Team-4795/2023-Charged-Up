// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import frc.robot.Commands.FaceAngle;
import frc.robot.Commands.TapeAlign;
import org.photonvision.PhotonCamera;
import frc.robot.Constants.VisionConstants;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final EndEffectorIntake m_intake = new EndEffectorIntake();;
  private final LiftArm m_arm = new LiftArm();
  private final Vision m_Vision = new Vision();
  private final PhotonCamera m_camera = new PhotonCamera(VisionConstants.SnakeEyesCamera);



  // The driver's controller
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_operatorController = new GenericHID(OIConstants.kOperatorControllerPort);

  // State manager
  StateManager m_manager = new StateManager();
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */   
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(1), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(4), OIConstants.kDriveDeadband),
                true,true),
            m_robotDrive));

    m_intake.setDefaultCommand(
        new RunCommand(
            () -> {
                m_intake.intakeAutomatic();

                m_intake.extended = m_intake.extendedTarget;
                
                if (m_arm.setpoint < ArmConstants.kLowWristLimit) {
                    m_intake.extended = false;
                }

                if (m_arm.setpoint > ArmConstants.kHighWristLimit) {
                    m_intake.extended = true;
                }

                if (m_intake.extended) {
                    m_intake.extend();
                } else {
                    m_intake.retract();
                }
            },
            m_intake
        )
    
    );


    // Axis 2 = to battery, axis 3 = away
    // Subtract up movement by down movement so they cancell out if both are pressed at once
    // Max speed is number multiplying this
    m_arm.setDefaultCommand(
        new RunCommand(
            () -> {
                double up = MathUtil.applyDeadband(m_driverController.getRawAxis(3), OIConstants.kArmDeadband);
                double down = MathUtil.applyDeadband(m_driverController.getRawAxis(2), OIConstants.kArmDeadband);
                
                // Get amount to change the setpoint
                double change = OIConstants.kArmManualSpeed * (Math.pow(up, 3) - Math.pow(down, 3));

                // New setpoint to move to
                double new_setpoint = m_arm.setpoint + change;

                // Manual soft limits, probably should remove
                if (new_setpoint < 0.14) {
                    new_setpoint = 0.14;
                } else if (new_setpoint > 0.98) {
                    new_setpoint = 0.98;
                }

                // Set new arm setpoint and move to it
                m_arm.setPosition(new_setpoint);
            },
            m_arm
        )
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link GenericHID}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */


  
  private void configureButtonBindings() {
    // A, B
    final JoystickButton setxbutton = new JoystickButton(m_driverController, 1);
    final JoystickButton resetheadingButton = new JoystickButton(m_driverController, 2);

    //face angle buttons
    final JoystickButton forwardbutton = new JoystickButton(m_driverController, 1);
    final JoystickButton lefttbutton = new JoystickButton(m_driverController, 2);
    final JoystickButton rightbutton = new JoystickButton(m_driverController, 3);
    final JoystickButton backbutton = new JoystickButton(m_driverController, 4);

    //vision align button
    final POVButton TapeAlign = new POVButton(m_driverController, 90);


    // Intake triggers
    final Trigger reverseIntake = new Trigger(() -> m_driverController.getPOV() == 90);
    final Trigger intake = new Trigger(() -> m_driverController.getPOV() == 270);
    // final JoystickButton reverseIntake = new JoystickButton(m_driverController, 8);
    // final JoystickButton intake = new JoystickButton(m_driverController, 7);

    // Keybinds:
    // https://docs.google.com/document/d/170FNOZ3DKwVowGESMP2AQLjpuYHfxeh4vl-hTgFNpbM/edit?usp=sharing

    // Left, right bumper
    final JoystickButton pickCone = new JoystickButton(m_operatorController, 5);
    final JoystickButton pickCube = new JoystickButton(m_operatorController, 6);

    // Handle dpad inputs
    final Trigger povUp = new Trigger(() -> m_operatorController.getPOV() == 0);
    final Trigger povLeft = new Trigger(() -> m_operatorController.getPOV() == 270);
    final Trigger povDown = new Trigger(() -> m_operatorController.getPOV() == 180);
    final Trigger povRight = new Trigger(() -> m_operatorController.getPOV() == 90);

    // X, Y
    final JoystickButton extend = new JoystickButton(m_driverController, 3);
    final JoystickButton retract = new JoystickButton(m_driverController, 4);

    // left, right bumper
    final JoystickButton isStoring = new JoystickButton(m_driverController, 6);
    final JoystickButton isNotStoring = new JoystickButton(m_driverController, 5);

    pickCone.onTrue(new InstantCommand(m_manager::pickCone, m_arm));
    pickCube.onTrue(new InstantCommand(m_manager::pickCube, m_arm));

    // Handle dpad triggers
    povUp.onTrue(new InstantCommand(() -> {m_manager.handleDpad(0); setStates();}, m_arm));
    povLeft.onTrue(new InstantCommand(() -> {m_manager.handleDpad(270); setStates();}, m_arm));
    povDown.onTrue(new InstantCommand(() -> {m_manager.handleDpad(180); setStates();}, m_arm));
    povRight.onTrue(new InstantCommand(() -> {m_manager.handleDpad(90); setStates();}, m_arm));

    isStoring.onTrue(new InstantCommand(m_manager::setStoring, m_arm));
    isNotStoring.onTrue(new InstantCommand(m_manager::setNotStoring, m_arm));

    extend.onTrue(new InstantCommand(
        () -> m_intake.setExtendedTarget(true),
        m_intake));

    retract.onTrue(new InstantCommand(
        () -> m_intake.setExtendedTarget(false),
        m_intake));

    setxbutton.whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));

    resetheadingButton.whileTrue(new RunCommand(m_robotDrive::zeroHeading));

    //Intake
    intake.whileTrue(new RunCommand(
        () -> m_intake.intake(DriveConstants.kIntakeSpeed),
        m_intake));
    
    reverseIntake.whileTrue(new RunCommand(
        () -> m_intake.intake(DriveConstants.kOuttakeSpeed),
        m_intake));

    //face angle
    forwardbutton.whileTrue(new FaceAngle(m_robotDrive, 
    ()-> m_driverController.getRawAxis(0),
    ()-> m_driverController.getRawAxis(1),
    0));

    lefttbutton.whileTrue(new FaceAngle(m_robotDrive, 
    ()-> m_driverController.getRawAxis(0),
    ()-> m_driverController.getRawAxis(1),
    -90));

    rightbutton.whileTrue(new FaceAngle(m_robotDrive, 
    ()-> m_driverController.getRawAxis(0),
    ()-> m_driverController.getRawAxis(1),
    90));

    backbutton.whileTrue(new FaceAngle(m_robotDrive, 
    ()-> m_driverController.getRawAxis(0),
    ()-> m_driverController.getRawAxis(1),
    180));

    //vision align
    TapeAlign.whileTrue(new TapeAlign(m_robotDrive, m_Vision, m_camera));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));

  }

  private void setStates() {
    m_manager.getArmSetpoint().ifPresent(m_arm::setPosition);
    m_manager.getIntakeSetpoint().ifPresent(m_intake::setIntakeSpeed);
    // m_manager.getWristExtended().ifPresent(m_intake::setExtended);
  }
}
