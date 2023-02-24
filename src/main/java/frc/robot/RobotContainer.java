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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControlContants;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import java.util.ResourceBundle.Control;

import javax.naming.ldap.ControlFactory;

import frc.robot.Commands.TapeAlign;
// import frc.robot.Constants.VisionConstants;
// import edu.wpi.first.wpilibj2.command.button.POVButton;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final EndEffectorIntake m_intake = new EndEffectorIntake();
  private final LiftArm m_arm = new LiftArm();
  private final Vision m_Vision = new Vision();

  // State manager
  StateManager m_manager = new StateManager(m_Vision, m_arm, m_intake);

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
                MathUtil.applyDeadband(ControlContants.driverController.getRawAxis(ControlContants.kDriveXSpeedAxis), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-ControlContants.driverController.getRawAxis(ControlContants.kDriveYSpeedAxis), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-ControlContants.driverController.getRawAxis(ControlContants.kDriveRotationAxis), OIConstants.kDriveDeadband),
                true,true),
            m_robotDrive));

    m_intake.setDefaultCommand(
        new RunCommand(
            () -> {
                m_intake.intakeFromGamepiece(m_manager.getGamepiece());

                m_intake.extended = m_intake.extendedTarget;

                if (m_arm.setpoint < ArmConstants.kLowWristLimit) {
                    m_intake.extended = false;
                }

                if (m_arm.setpoint > ArmConstants.kHighWristLimit) {
                    m_intake.extended = false;
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


    // Subtract up movement by down movement so they cancel out if both are pressed at once
    m_arm.setDefaultCommand(
        new RunCommand(
            () -> {
                double up = MathUtil.applyDeadband(ControlContants.driverController.getRawAxis(ControlContants.kArmDownAxis), OIConstants.kArmDeadband);
                double down = MathUtil.applyDeadband(ControlContants.driverController.getRawAxis(ControlContants.kArmDownAxis), OIConstants.kArmDeadband);
                
                // Get amount to change the setpoint
                double change = OIConstants.kArmManualSpeed * (Math.pow(up, 3) - Math.pow(down, 3));

                // New setpoint to move to
                double new_setpoint = m_arm.setpoint + change;

                // Manual soft limits, probably should remove
                if (new_setpoint < ArmConstants.kLowSetpointLimit) {
                    new_setpoint = ArmConstants.kLowSetpointLimit;
                } else if (new_setpoint > ArmConstants.kHighSetpointLimit) {
                    new_setpoint = ArmConstants.kHighSetpointLimit;
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
    // Pick cone, cube
    ControlContants.operatorBumperLeft.onTrue(new InstantCommand(m_manager::pickCone, m_arm));
    ControlContants.operatorBumperRight.onTrue(new InstantCommand(m_manager::pickCube, m_arm));

    // Setpoints
    ControlContants.operatorDpadUp.onTrue(new InstantCommand(() -> m_manager.dpadUp(), m_arm));
    ControlContants.operatorDpadLeft.onTrue(new InstantCommand(() -> m_manager.dpadLeft(), m_arm));
    ControlContants.operatorDpadDown.onTrue(new InstantCommand(() -> m_manager.dpadDown(), m_arm));
    ControlContants.operatorDpadRight.onTrue(new InstantCommand(() -> m_manager.dpadRight(), m_arm));

    // HiLetGo override
    ControlContants.operatorA.onTrue(new InstantCommand(() -> m_intake.overrideStoring(true)));
    ControlContants.operatorA.onFalse(new InstantCommand(() -> m_intake.overrideStoring(false)));

    // Set x
    ControlContants.driverA.whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));

    // Reset heading
    ControlContants.driverB.whileTrue(new RunCommand(m_robotDrive::zeroHeading));
    
    // Outtake
    ControlContants.operatorDpadRight.whileTrue(new RunCommand(
        m_intake::outtake,
        m_intake));

    // Pneumatic override
    ControlContants.operatorX.whileTrue(new RunCommand(
        () -> m_intake.extend(),
        m_intake));

    ControlContants.operatorY.whileTrue(new RunCommand(
        () -> m_intake.retract(),
        m_intake));

    // Vision align
    ControlContants.driverX.whileTrue(new TapeAlign(
        m_robotDrive,
        m_Vision,
        () -> ControlContants.driverController.getRawAxis(ControlContants.kAlignXSpeedAxis),
        () -> -ControlContants.driverController.getRawAxis(ControlContants.kAlignYSpeedAxis)
    ));
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
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    return new InstantCommand();

  }
}
