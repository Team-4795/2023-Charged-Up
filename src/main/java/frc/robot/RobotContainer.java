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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Commands.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControlContants;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import java.nio.channels.spi.AbstractSelector;
import java.util.List;
import java.util.ResourceBundle.Control;

import javax.imageio.plugins.tiff.GeoTIFFTagSet;
import javax.naming.ldap.ControlFactory;

import frc.robot.Commands.TapeAlign;
// import frc.robot.Constants.VisionConstants;
// import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  private final EndEffectorIntake m_intake = new EndEffectorIntake();
  private final LiftArm m_arm = new LiftArm();
  private final Vision m_Vision = new Vision();
  private final LEDs m_led = new LEDs();


  // The driver's controller
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_operatorController = new GenericHID(OIConstants.kOperatorControllerPort);

  // State manager
  StateManager m_manager = new StateManager(m_Vision, m_arm, m_intake, m_led, m_robotDrive);

  AutoSelector autoSelector = new AutoSelector(m_robotDrive, m_intake, m_arm, m_robotDrive.m_field, m_manager, m_Vision);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    autoSelector = new AutoSelector(m_robotDrive, m_intake, m_arm,  m_robotDrive.m_field, m_manager, m_Vision);

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
                m_intake.intakeFromGamepiece(m_manager.getGamepiece(), m_manager.isStowing());

                m_intake.extended = m_intake.extendedTarget;

                if (m_arm.getPosition() < ArmConstants.kLowWristLimit) {
                    m_intake.extended = false;
                }

                if (m_arm.getPosition() > ArmConstants.kHighWristLimit) {
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
                double up = MathUtil.applyDeadband(ControlContants.operatorController.getRawAxis(ControlContants.kArmUpAxis), OIConstants.kArmDeadband);
                double down = MathUtil.applyDeadband(ControlContants.operatorController.getRawAxis(ControlContants.kArmDownAxis), OIConstants.kArmDeadband);
                
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
                m_arm.setTargetPosition(new_setpoint);

                m_arm.runAutomatic();
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
    ControlContants.operatorBumperLeft.onTrue(new InstantCommand(m_manager::pickCone, m_arm, m_Vision, m_led));
    ControlContants.operatorBumperRight.onTrue(new InstantCommand(m_manager::pickCube, m_arm, m_Vision, m_led));

    // Setpoints
    final JoystickButton balanceButton = new JoystickButton(m_driverController, 4);

    balanceButton.whileTrue(new SequentialCommandGroup(
        new DriveCommandOld(m_robotDrive, -AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold, AutoConstants.checkDuration).withTimeout(AutoConstants.overrideDuration),
        new AutoBalanceOld(m_robotDrive, AutoConstants.angularVelocityErrorThreshold)
    ));

    ControlContants.operatorDpadUp.onTrue(new InstantCommand(m_manager::dpadUp, m_arm));
    ControlContants.operatorDpadLeft.onTrue(new InstantCommand(m_manager::dpadLeft, m_arm));
    ControlContants.operatorDpadDown.onTrue(new InstantCommand(m_manager::dpadDown, m_arm));
    ControlContants.operatorDpadRight.onTrue(new InstantCommand(m_manager::dpadRight, m_arm));

    // HiLetGo override
    ControlContants.operatorA.onTrue(new InstantCommand(() -> m_intake.setOverrideStoring(true)));
    ControlContants.operatorA.onFalse(new InstantCommand(() -> m_intake.setOverrideStoring(false)));

    // Set x
    ControlContants.driverBumperLeft.whileTrue(new RunCommand(
        m_robotDrive::setX,
        m_robotDrive));

    // Reset heading
    ControlContants.driverBumperRight.whileTrue(new RunCommand(m_robotDrive::zeroHeading));
    
    // Outtake
    ControlContants.driverDpadRight.whileTrue(
    new RunCommand(
        () -> {
            m_intake.outtake();
            switch (m_manager.getState()) {
                case MidScore: switch (m_manager.getGamepiece()) {
                    case Cone: m_intake.extend();
                    default: break;
                };
                default: break;
            }
        },
        m_intake));

    // Pneumatic override
    ControlContants.operatorX.whileTrue(new RunCommand(
        m_intake::extend,
        m_intake));

    ControlContants.operatorY.whileTrue(new RunCommand(
        m_intake::retract,
        m_intake));

    // Vision align
    ControlContants.driverDpadLeft.whileTrue(new TapeAlign(
        m_robotDrive,
        m_Vision,
        () -> ControlContants.driverController.getRawAxis(ControlContants.kAlignXSpeedAxis),
        () -> -ControlContants.driverController.getRawAxis(ControlContants.kAlignYSpeedAxis)
    ));

    // reset LEDs when were not targeting
    // new Trigger(m_intake::isStoring).onTrue(new InstantCommand(m_led::reset, m_led));
  }

  public void setDriverRumble(double rumble) {
    ControlContants.driverController.setRumble(RumbleType.kLeftRumble, rumble);
    ControlContants.driverController.setRumble(RumbleType.kRightRumble, rumble);
  }

  public void setOperatorRumble(double rumble) {
    ControlContants.operatorController.setRumble(RumbleType.kLeftRumble, rumble);
    ControlContants.operatorController.setRumble(RumbleType.kRightRumble, rumble);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return autoSelector.getSelected();
  }

  public void setNotStoring() {
    m_intake.setOverrideStoring(false);
  }
}
