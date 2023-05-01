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
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.Arm;
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
  public Drive m_robotDrive;
  private final EndEffectorIntake m_intake = new EndEffectorIntake();
  private final Wrist m_wrist = new Wrist();
  private Arm m_arm;
  private final Vision m_Vision = new Vision();
  private final LEDs m_led = new LEDs();
  private final LandingGear m_landing = new LandingGear();
  private final Rollerbar m_rollerbar = new Rollerbar();


  // The driver's controller
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_operatorController = new GenericHID(OIConstants.kOperatorControllerPort);

  // State manager
  StateManager m_manager;

  AutoSelector autoSelector;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.getRobot()) {
        case Comp: 
            m_arm = new Arm(new ArmIOSparkMax()); 
            m_robotDrive = new Drive(
                new GyroIOSim(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3)
            );
            break;
        case Sim: 
            m_arm = new Arm(new ArmIOSim());
            m_robotDrive = new Drive(
                new GyroIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim()
            );
            break;
    }

    m_manager = new StateManager(m_Vision, m_arm, m_intake, m_led, m_robotDrive, m_wrist, m_rollerbar);

    // Configure the button bindings
    autoSelector = new AutoSelector(m_robotDrive, m_intake, m_arm,  m_robotDrive.m_field, m_manager, m_Vision, m_wrist, m_rollerbar); // huh? see above

    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-ControlConstants.driverController.getRawAxis(ControlConstants.kDriveXSpeedAxis), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(ControlConstants.driverController.getRawAxis(ControlConstants.kDriveYSpeedAxis), OIConstants.kDriveDeadband),
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
            boolean target = m_wrist.extendedTarget;

            if (m_arm.getPosition() < ArmConstants.kLowWristLimit) {
                target = false;
            }

            if (m_arm.getPosition() > ArmConstants.kHighWristLimit) {
                target = false;
            }

            if (target) {
                m_wrist.extend();
            } else {
                m_wrist.retract();
            }
        }, 
        m_wrist)
    );

    m_led.setDefaultCommand(
        new RunCommand(() -> {

            if (EndEffectorIntake.isStoring()) {
                m_led.setBottomRGB(0, 255, 0);
            } else {
                m_led.setBottomRGB(255, 0, 0);
            }

            if (StateManager.getGamepiece() == Gamepiece.Cube) {
                m_led.setTopRGB(127, 0, 255);
            } else {
                m_led.setTopRGB(255, 255, 0);
            }
        }
        , m_led)
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

                m_arm.setTargetPosition(new_setpoint);
            },
            m_arm
        )
    );

    m_rollerbar.setDefaultCommand(new RunCommand(
        () -> {
            // If the rollerbar isnt where we want it to be, and if its not safe to move rollerbar, then
            // set the arm target position to be just above the minimum height to extend/retract it.
            // Otherwise, if the rollerbar is where we want it to be, move back to the correct setpoint.
            if (m_rollerbar.isExtended() != m_rollerbar.getTarget()) {
                if (!m_rollerbar.safeToMove(m_arm.setpoint)) {
                    m_arm.setTargetPosition(RollerbarConstants.kArmBoundary + 0.025);
                    m_arm.isTemporary = true;
                }
            } else if (m_arm.isTemporary) {
                m_manager.setSetpoints();
                m_arm.isTemporary = false;
            }

            m_rollerbar.tryMove(m_arm.getPosition());

            if (!EndEffectorIntake.isStoring() && m_rollerbar.isExtended() && m_arm.atSetpoint()) {
                m_rollerbar.spin();
            } else {
                m_rollerbar.stop();
            }

        },
        m_rollerbar));
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
    ControlConstants.operatorA.whileTrue(new RunCommand(m_rollerbar::reverse, m_rollerbar));
    ControlConstants.operatorB.onTrue(new RunCommand(m_rollerbar::spin, m_rollerbar));
    ControlConstants.driverA.onTrue(new InstantCommand(m_rollerbar::toggle));
    // ControlConstants.operatorA
    // .whileTrue(new RunCommand(m_rollerbar::spin, m_rollerbar))
    // .whileFalse(new RunCommand(m_rollerbar::stop, m_rollerbar));
    
    ControlConstants.operatorDpadRight.onTrue(new InstantCommand(m_manager::dpadRight, m_arm));
    ControlConstants.operatorDpadDown.onTrue(new InstantCommand(m_manager::dpadDown, m_arm));

    ControlConstants.operatorY
        .onTrue(new InstantCommand(() -> m_intake.setOverrideStoring(true)))
        .onFalse(new InstantCommand(() -> m_intake.setOverrideStoring(false)));

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

    ControlConstants.driverY.onTrue(new InstantCommand(() -> m_landing.setTargetExtended(!m_landing.getTargetExtended())));
 
    new Trigger(EndEffectorIntake::isStoring)
        .debounce(0.1)
        .onTrue(new ParallelCommandGroup(
            new RunCommand(() -> setRumble(0.25))
                .withTimeout(0.5)
                .andThen(new InstantCommand(() -> setRumble(0))),
            new LEDCommand(m_led, () -> -1.0).withTimeout(1.0)
        ));
    
    // reset LEDs when were not targeting
    // new Trigger(m_intake::isStoring).onTrue(new InstantCommand(m_led::reset, m_led));
  
    new Trigger(() -> Math.abs(MathUtil.applyDeadband(ControlConstants.operatorController.getRawAxis(5), 0.1)) > 0)
        .whileTrue(new LEDCommand(
                m_led, 
                () -> MathUtil.applyDeadband(ControlConstants.operatorController.getRawAxis(5), 0.1)
            )
        );
}

  public void setDriverRumble(double rumble) {
    ControlConstants.driverController.setRumble(RumbleType.kLeftRumble, rumble);
    ControlConstants.driverController.setRumble(RumbleType.kRightRumble, rumble);
  }

  public void setOperatorRumble(double rumble) {
    ControlConstants.operatorController.setRumble(RumbleType.kLeftRumble, rumble);
    ControlConstants.operatorController.setRumble(RumbleType.kRightRumble, rumble);
  }

  public void setRumble(double rumble) {
    setDriverRumble(rumble);
    setOperatorRumble(rumble);
  }

  public void cancelOverride(){
    m_intake.setOverrideStoring(false);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return autoSelector.getSelected();
  }

  public void resetArm() {
      m_arm.resetPosition();
  }
}
