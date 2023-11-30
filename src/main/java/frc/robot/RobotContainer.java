// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.LEDCommand;
import frc.robot.Commands.TapeAlign;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.motorizedWrist.Wrist;
import frc.robot.subsystems.rollerbar.Rollerbar;
import frc.utils.ConstraintManager;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    AutoSelector autoSelector;

    public RobotContainer() {
        // Configure the button bindings
        autoSelector = new AutoSelector();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        Arm arm = Arm.getInstance();
        // Drive drive = Drive.getInstance();
        Swerve drive = Swerve.getInstance();
        Rollerbar rollerbar = Rollerbar.getInstance();
        Intake intake = Intake.getInstance();
        Wrist wrist = Wrist.getInstance();
        StateManager manager = StateManager.getInstance();
        ConstraintManager.initialize();

        // OIConstants.operatorController.back().onTrue(Commands.runOnce(drive::resetEstimatedPose));
        OIConstants.operatorController.button(6).onTrue(new PrintCommand("Worked"));

        OIConstants.operatorController.a().whileTrue(Commands.run(rollerbar::reverse, rollerbar));
        OIConstants.operatorController.b().onTrue(Commands.run(rollerbar::spin, rollerbar));
        OIConstants.operatorController.x().onTrue(Commands.runOnce(wrist::flip));
        OIConstants.operatorController
                .y()
                .onTrue(Commands.runOnce(() -> intake.setOverrideStoring(true)))
                .onFalse(Commands.runOnce(() -> intake.setOverrideStoring(false)));

        OIConstants.operatorController.leftBumper().onTrue(Commands.runOnce(manager::pickCone));
        OIConstants.operatorController.rightBumper().onTrue(Commands.runOnce(manager::pickCube));

        OIConstants.operatorController.povUp().onTrue(Commands.runOnce(manager::dpadUp));
        OIConstants.operatorController.povLeft().onTrue(Commands.runOnce(manager::dpadLeft));
        OIConstants.operatorController.povRight().onTrue(Commands.runOnce(manager::dpadRight));
        OIConstants.operatorController.povDown().onTrue(Commands.runOnce(manager::dpadDown));
        // Force rollerbar toggle
        OIConstants.driverController.a().onTrue(Commands.runOnce(rollerbar::toggle));

        // Landing gear toggle
        OIConstants.driverController.y()
                .onTrue(Commands.runOnce(wrist::toggleBinaryControl));
        
        OIConstants.driverController.rightBumper().onTrue(Commands.runOnce(() -> drive.zeroHeading()));
        // Outtake
        OIConstants.driverController
                .povRight()
                .whileTrue(Commands.either(
                        Commands.sequence(
                                Commands.runOnce(() -> wrist.setExtendedTarget(false), wrist),
                                new WaitCommand(IntakeConstants.kFlickTime),
                                Commands.run(intake::outtake, intake)),
                        Commands.run(intake::outtake, intake),
                        () -> {
                            switch (manager.getState()) {
                                case BackwardsHighScore:
                                    switch (StateManager.getInstance().getGamepiece()) {
                                        case Cube:
                                            return true;
                                        default:
                                            return false;
                                    }
                                default:
                                    return false;
                            }
                        }));

                        
        // Vision align
        OIConstants.driverController
                .povLeft()
                .whileTrue(new TapeAlign(
                        () -> OIConstants.driverController.getLeftX(), () -> -OIConstants.driverController.getLeftY()));

        // Intaking LED trigger
        new Trigger(Intake::isStoring)
                .debounce(0.1)
                .onTrue(Commands.parallel(
                        Commands.run(() -> setRumble(0.25))
                                .withTimeout(0.5)
                                .andThen(Commands.runOnce(() -> setRumble(0))),
                        new LEDCommand(() -> -1.0).withTimeout(1.0)));

        // Operator LED trigger
        // new Trigger(() -> Math.abs(MathUtil.applyDeadband(OIConstants.driverController.getRightY(), 0.1)) > 0)
        //         .whileTrue(new LEDCommand(() -> MathUtil.applyDeadband(OIConstants.driverController.getRightY(), 0.1)));
    }

    public void setDriverRumble(double rumble) {
        OIConstants.driverController.getHID().setRumble(RumbleType.kBothRumble, rumble);
    }

    public void setOperatorRumble(double rumble) {
        OIConstants.operatorController.getHID().setRumble(RumbleType.kBothRumble, rumble);
    }

    public void setRumble(double rumble) {
        setDriverRumble(rumble);
        setOperatorRumble(rumble);
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }
}
