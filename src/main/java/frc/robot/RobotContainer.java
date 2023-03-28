// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Commands.*;
import frc.robot.Constants.*;
import frc.robot.StateManager.Gamepiece;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


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
  private final Wrist m_wrist = new Wrist();
  private final LiftArm m_arm = new LiftArm();
  private final Vision m_Vision = new Vision();
  private final LEDs m_led = new LEDs();
  private final LandingGear m_landing = new LandingGear();


  // The driver's controller
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_operatorController = new GenericHID(OIConstants.kOperatorControllerPort);

  // State manager
  StateManager m_manager = new StateManager(m_Vision, m_arm, m_intake, m_led, m_robotDrive, m_wrist);

  AutoSelector autoSelector;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    autoSelector = new AutoSelector(m_robotDrive, m_intake, m_arm,  m_robotDrive.m_field, m_manager, m_Vision, m_wrist); // huh? see above

    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(ControlConstants.driverController.getRawAxis(ControlConstants.kDriveXSpeedAxis), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-ControlConstants.driverController.getRawAxis(ControlConstants.kDriveYSpeedAxis), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-ControlConstants.driverController.getRawAxis(ControlConstants.kDriveRotationAxis), OIConstants.kDriveDeadband),
                true,true),
            m_robotDrive));

    m_intake.setDefaultCommand(
        new RunCommand(
            () -> {
                m_intake.intakeFromGamepiece(m_manager.isStowing());
            }, m_intake
        )
    );

    m_landing.setDefaultCommand(
        new RunCommand(() -> {
            if(m_landing.getExtended() != m_landing.getTargetExtended()){
                m_landing.updateExtended();
            }
        }
        , m_landing)
    );

    m_wrist.setDefaultCommand(
        new RunCommand(() -> {
            m_wrist.extended = m_wrist.extendedTarget;

            if (m_arm.getPosition() < ArmConstants.kLowWristLimit) {
                m_wrist.extended = false;
            }

            if (m_arm.getPosition() > ArmConstants.kHighWristLimit) {
                m_wrist.extended = false;
            }

            if (m_wrist.extended) {
                m_wrist.extend();
            } else {
                m_wrist.retract();
            }
        }, 
        m_wrist)
    );

    

    // Subtract up movement by down movement so they cancel out if both are pressed at once
    m_arm.setDefaultCommand(
        new RunCommand(
            () -> {
                double up = MathUtil.applyDeadband(ControlConstants.operatorController.getRawAxis(ControlConstants.kArmUpAxis), OIConstants.kArmDeadband);
                double down = MathUtil.applyDeadband(ControlConstants.operatorController.getRawAxis(ControlConstants.kArmDownAxis), OIConstants.kArmDeadband);
                
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

                // Set new arm setpoint
                if (new_setpoint != m_arm.setpoint) {
                    m_arm.setpoint = new_setpoint;
                    m_arm.runManual();
                } else {
                    m_arm.runAutomatic();
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
    // Pick cone, cube
    ControlConstants.operatorBumperLeft.onTrue(new InstantCommand(m_manager::pickCone, m_arm, m_Vision, m_led));
    ControlConstants.operatorBumperRight.onTrue(new InstantCommand(m_manager::pickCube, m_arm, m_Vision, m_led));

    ControlConstants.operatorDpadUp.onTrue(new InstantCommand(m_manager::dpadUp, m_arm));
    ControlConstants.operatorDpadLeft.onTrue(new InstantCommand(m_manager::dpadLeft, m_arm));
    ControlConstants.operatorDpadDown.onTrue(new InstantCommand(m_manager::dpadDown, m_arm));
    ControlConstants.operatorDpadRight.onTrue(new InstantCommand(m_manager::dpadRight, m_arm));

    ControlConstants.operatorY
        .onTrue(new InstantCommand(() -> m_intake.setOverrideStoring(true)))
        .onFalse(new InstantCommand(() -> m_intake.setOverrideStoring(false)));

    ControlConstants.operatorB.onTrue(new InstantCommand(m_manager::stowHigh, m_arm));

    // Set x
    ControlConstants.driverBumperLeft.whileTrue(new RunCommand(
        m_robotDrive::setX,
        m_robotDrive));

    // Reset heading
    ControlConstants.driverBumperRight.whileTrue(new RunCommand(m_robotDrive::zeroHeading));
    
    // Outtake
    ControlConstants.driverDpadRight.whileTrue(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(m_wrist::retract, m_wrist),
                new WaitCommand(IntakeConstants.kFlickTime),
                new RunCommand(m_intake::outtake, m_intake)
            ),
            new RunCommand(m_intake::outtake, m_intake),
            () -> {
                switch (m_manager.getState()) {
                    case BackwardsHighScore: switch (StateManager.getGamepiece()) {
                        case Cube: return true;
                        default: return false;
                    }
                    default: return false;
                }
            })
    );
    // new RunCommand(
    //     () -> {
    //         m_intake.outtake();
    //         switch (m_manager.getState()) {
    //             case MidScore: switch (StateManager.getGamepiece()) {
    //                 case Cone: m_wrist.extend();
    //                 default: break;
    //             };
    //             default: break;
    //         }
    //     },
    //     m_intake, m_wrist));

    // Pneumatic override
    ControlConstants.operatorX.onTrue(new InstantCommand(m_wrist::flip, m_wrist));

    // Vision align
    ControlConstants.driverDpadLeft.whileTrue(new TapeAlign(
        m_robotDrive,
        m_Vision,
        () -> ControlConstants.driverController.getRawAxis(ControlConstants.kAlignXSpeedAxis),
        () -> -ControlConstants.driverController.getRawAxis(ControlConstants.kAlignYSpeedAxis)
    ));

    ControlConstants.driverX.onTrue(new InstantCommand(m_manager::stowHigh, m_arm));

    ControlConstants.driverX.whileTrue(new SequentialCommandGroup(
        new WaitCommand(0.4),
        new RunCommand(() -> {
        double change = OIConstants.kArmManualSpeed * (-0.75);
        double new_setpoint = m_arm.setpoint + change;

        if(new_setpoint <= ArmConstants.maxWindPoint){
            new_setpoint = ArmConstants.maxWindPoint;
        }

                // Set new arm setpoint
        if (new_setpoint != m_arm.setpoint) {
            m_arm.setpoint = new_setpoint;
            m_arm.runManual();
        }           
    }, m_arm)
    ));
    
    ControlConstants.driverX.onFalse(new SequentialCommandGroup(
                                            new Yeeeeet(m_arm, m_wrist, m_intake, m_manager, "cube"),
                                            new WaitCommand(0.3),
                                            new InstantCommand(m_manager::pickCone),
                                            new InstantCommand(m_manager::dpadRight),
                                            new RunCommand(m_arm::runAutomatic).withTimeout(1)
    ));

    ControlConstants.driverY.onTrue(new InstantCommand(() -> m_landing.setTargetExtended(!m_landing.getTargetExtended())));

    // reset LEDs when were not targeting
    // new Trigger(m_intake::isStoring).onTrue(new InstantCommand(m_led::reset, m_led));
 
    new Trigger(m_intake::isStoring).onTrue(new RunCommand(() -> setOperatorRumble(0.5)).withTimeout(0.5));

  }

  public void setDriverRumble(double rumble) {
    ControlConstants.driverController.setRumble(RumbleType.kLeftRumble, rumble);
    ControlConstants.driverController.setRumble(RumbleType.kRightRumble, rumble);
  }

  public void setOperatorRumble(double rumble) {
    ControlConstants.operatorController.setRumble(RumbleType.kLeftRumble, rumble);
    ControlConstants.operatorController.setRumble(RumbleType.kRightRumble, rumble);
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

  public void resetArm() {
      m_arm.resetPosition();
  }
}
