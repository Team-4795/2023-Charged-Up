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
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.EndEffectorIntake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
//<<<<<<< HEAD
  private final AutoSelector autoSelector;
//=======
  private final EndEffectorIntake m_intake = new EndEffectorIntake();;
  private final LiftArm m_arm = new LiftArm();
//>>>>>>> main

  // The driver's controller
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_operatorController = new GenericHID(OIConstants.kOperatorControllerPort);

  StateManager m_manager = new StateManager(m_arm);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    autoSelector = new AutoSelector(m_robotDrive);

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
            () -> m_intake.intake(0), // Change to DriveConstants.kSlowIntakeSpeed
            m_intake
        )
    );

    // Axis 2 = to battery, axis 3 = away
    // Subtract up movement by down movement so they cancell out if both are pressed at once
    // Max speed is number multiplying this
    m_arm.setDefaultCommand(
        new RunCommand(
            () -> {
                double up = m_operatorController.getRawAxis(3);
                double down = m_operatorController.getRawAxis(2);

                if (up != 0 && down != 0) {
                    m_arm.move(0.3 * (Math.pow(up, 3) - Math.pow(down, 3)));
                } 
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

    final JoystickButton setxbutton = new JoystickButton(m_driverController, 5);
    final JoystickButton resetheadingButton = new JoystickButton(m_driverController, 6);

    //Intake dpad
    // final Trigger reverseIntake = new Trigger(() -> m_operatorController.getPOV()==90);
    // final Trigger intake = new Trigger(() -> m_operatorController.getPOV()==270);
    final JoystickButton reverseIntake = new JoystickButton(m_driverController, 8);
    final JoystickButton intake = new JoystickButton(m_driverController, 7);

    // this is only for testing will convert to d-pad with 2 modes or something else
    //  final JoystickButton stowed = new JoystickButton(m_driverController, some number);
    //  final JoystickButton intakeCube = new JoystickButton(m_driverController, some number);
    //  final JoystickButton highFeeder = new JoystickButton(m_driverController, some number);
    //  final JoystickButton lowFeeder = new JoystickButton(m_driverController, some number);

    // Left, right bumper
    final JoystickButton pickCone = new JoystickButton(m_operatorController, 5);
    final JoystickButton pickCube = new JoystickButton(m_operatorController, 6);

    // D-Pad in this order: top, left, bottom, right
    final Trigger stow = new Trigger(() -> m_operatorController.getPOV() == 0);
    final Trigger button1 = new Trigger(() -> m_operatorController.getPOV() == 270); // Low pickup or low score
    final Trigger button2 = new Trigger(() -> m_operatorController.getPOV() == 180); // Single feeder or mid score
    final Trigger button3 = new Trigger(() -> m_operatorController.getPOV() == 90); // Double feeder or high score

    // A, B
    final JoystickButton isStoring = new JoystickButton(m_operatorController, 1);
    final JoystickButton isNotStoring = new JoystickButton(m_operatorController, 2);

    pickCone.onTrue(new RunCommand(m_manager::pickCone));
    pickCube.onTrue(new RunCommand(m_manager::pickCube));

    stow.onTrue(new RunCommand(m_manager::stow));
    button1.onTrue(new RunCommand(m_manager::button1));
    button2.onTrue(new RunCommand(m_manager::button2));
    button3.onTrue(new RunCommand(m_manager::button3));

    isStoring.onTrue(new RunCommand(m_manager::setStoring));
    isNotStoring.onTrue(new RunCommand(m_manager::setNotStoring));

    // Temporary toggle storing button
    // toggleStoring.onTrue(new RunCommand(m_manager::toggleStoring));

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

    // lowGoal.onTrue(new RunCommand(() -> m_arm.setPosition(0.2)));
    // highGoal.onTrue(new RunCommand(() -> m_arm.setPosition(0.4)));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoSelector.getSelected();

  }
}
