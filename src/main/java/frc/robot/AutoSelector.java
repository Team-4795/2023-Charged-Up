package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.StateManager.State;
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.ChangeStateCommand;
import frc.robot.Commands.DriveCommandOld;
import frc.robot.Commands.TapeAlign;
import frc.robot.autoPaths.BalanceCubeTwoGamePiece;
import frc.robot.autoPaths.CableAutoBalance;
import frc.robot.autoPaths.CableCubeTwoGamePiece;
import frc.robot.autoPaths.CenterScoreBalance;
import frc.robot.autoPaths.FreeAutoBalance;
import frc.robot.autoPaths.FreeCubeTwoGamePiece;
import frc.robot.autoPaths.FreeGrabBalance;
import frc.robot.autoPaths.FreeGrabCommunity;
import frc.robot.autoPaths.Shooooot;
import frc.robot.autoPaths.SimpleHighCube;
import frc.robot.autoPaths.SimpleMidCone;
import frc.robot.autoPaths.TwoScoreOnePickup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class AutoSelector {

    private final SendableChooser < Command > chooser = new SendableChooser < > ();

    DriveSubsystem drivebase;
    EndEffectorIntake m_intake;
    LiftArm m_arm;
    Field2d m_field;
    StateManager m_manager;
    Vision m_vision;
    Wrist wrist;

    // Define command which scores gamepeieces at various setpoints
    // only the first command is documented
    public Command score(String gamepeice, String setpoint) {

        // No longer need because of current sensing
        // new InstantCommand(() -> m_intake.setOverrideStoring(true)),

        if (gamepeice.equals("cube")) {
            m_manager.pickCube();

            if (setpoint.equals("high")) {
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        // Move to high score setpoint depending on gyro
                        new ChangeStateCommand(true, m_manager.highScore(), m_manager, m_arm, m_intake, wrist),
                        // Meanwhile auto align
                        new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                m_vision.switchToTag();
                            }),
                            new TapeAlign(
                                drivebase, m_vision,
                                () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))),
                    new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0)
                );
            } else if (setpoint.equals("mid")) {
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> m_manager.dpadLeft(), m_arm),
                            new InstantCommand(wrist::retract, wrist),
                            new RunCommand(m_arm::runAutomatic, m_arm).withTimeout(1.5)),
                        // meanwhile auto align
                        new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                m_vision.switchToTag();
                            }),
                            new TapeAlign(
                                drivebase, m_vision,
                                () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))),
                    new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
                    new InstantCommand(wrist::retract, wrist)
                );
            } else if (setpoint.equals("low")) {
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> m_manager.dpadDown(), m_arm),
                            new InstantCommand(wrist::retract, wrist),
                            new RunCommand(m_arm::runAutomatic, m_arm).withTimeout(1.5)),
                        // meanwhile auto align
                        new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                m_vision.switchToTag();
                            }),
                            new TapeAlign(
                                drivebase, m_vision,
                                () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))),
                    new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
                    new InstantCommand(wrist::retract, wrist)
                );
            }
        } else if (gamepeice.equals("cone")) {
            m_manager.pickCone();
            if (setpoint.equals("mid")) {
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> m_manager.dpadLeft(), m_arm),
                            new InstantCommand(wrist::retract, wrist),
                            new RunCommand(m_arm::runAutomatic, m_arm).withTimeout(1.5)),
                        // meanwhile auto align
                        new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                m_vision.switchToTape();
                            }),
                            new TapeAlign(
                                drivebase, m_vision,
                                () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))),
                    new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
                    new InstantCommand(wrist::retract, wrist)
                );
            } else if (setpoint.equals("low")) {
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> m_manager.dpadDown(), m_arm),
                            new InstantCommand(wrist::retract, wrist),
                            new RunCommand(m_arm::runAutomatic, m_arm).withTimeout(1.5)),
                        // meanwhile auto align
                        new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                m_vision.switchToTape();
                            }),
                            new TapeAlign(
                                drivebase, m_vision,
                                () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))),
                    new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
                    new InstantCommand(wrist::retract, wrist)
                );
            }
        }

        return null;

    }

    public Command scoreV2(String gamepiece, String setpoint) {
        State newState;

        // Set gamepiece
        switch (gamepiece) {
            case "cube": m_manager.pickCube(); break;
            case "cone": m_manager.pickCone(); break;
            default: throw new IllegalArgumentException("\"" + gamepiece + "\" is not a valid gamepiece.");
        }

        // Get setpoint to move to
        switch (setpoint) {
            case "high": newState = m_manager.highScore(); break;
            case "mid": newState = m_manager.midScore(); break;
            case "low": newState = m_manager.lowScore(); break;
            default: throw new IllegalArgumentException("\"" + setpoint + "\" is not a valid setpoint.");
        }

        // Align while moving arm then outtake
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new TapeAlign(drivebase, m_vision, () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed).withTimeout(1),
                new ChangeStateCommand(true, newState, m_manager, m_arm, m_intake, wrist)
            ),
            new WaitCommand(0.25),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(0.5)
        );
    }

    public Command intake(String gamepeice) {
        if (gamepeice.equals("cube")) {
            return new SequentialCommandGroup(
                new InstantCommand(wrist::retract),
                new InstantCommand(m_manager::pickCube),
                new InstantCommand(() -> m_manager.dpadDown()),
                new RunCommand(m_arm::runAutomatic, m_arm).withTimeout(1.5),
                new RunCommand(() -> m_intake.runIntake(), m_intake)
                .withTimeout(1));

        } else if (gamepeice.equals("cone")) {
            return new SequentialCommandGroup(
                new InstantCommand(wrist::retract),
                new InstantCommand(m_manager::pickCone),
                new InstantCommand(() -> m_manager.dpadDown()),
                new WaitUntilCommand(m_arm::atSetpoint),
                new RunCommand(() -> m_intake.runIntake(), m_intake)
                .withTimeout(1));
        }

        return null;
    }

    public Command intakeV2(String gamepiece, double maxDuration) {
        // Set gamepiece
        switch (gamepiece) {
            case "cube": m_manager.pickCube(); break;
            case "cone": m_manager.pickCone(); break;
            default: throw new IllegalArgumentException("\"" + gamepiece + "\" is not a valid gamepiece.");
        }

        // Move arm and then run intake for maxDuration seconds
        return new SequentialCommandGroup(
            new ChangeStateCommand(true, State.LowPickup, m_manager, m_arm, m_intake, wrist),
            new RunCommand(() -> m_intake.runIntake(), m_intake)
                .withTimeout(maxDuration)
        );
    }

    public Command stow() {
        return new ChangeStateCommand(true, State.StowInFrame, m_manager, m_arm, m_intake, wrist);
    }

    public Command followTrajectory(PathPlannerTrajectory traj) {
        return drivebase.followTrajectoryCommand(traj);
    }

    public Command autoStartUp(PathPlannerTrajectory traj, boolean flip) {
        return drivebase.autoStartUp(traj, flip);
    }

    public Command autoBalanceOld() {
        return new SequentialCommandGroup(
            new DriveCommandOld(drivebase, -AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold,
                    AutoConstants.checkDuration).withTimeout(AutoConstants.overrideDuration),
            new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold)
        );
    }

    public Command tapeAlign() {
        return new TapeAlign(
            drivebase, m_vision,
            () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed);
    }

    public Command tapeAlignFast() {
        return new TapeAlign(
            drivebase, m_vision,
            () -> AutoConstants.VisionMoveFastX, () -> AutoConstants.VisionMoveFastY);
    }

    public AutoSelector(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
        StateManager m_manager, Vision m_vision, Wrist wrist) {

        this.drivebase = drivebase;
        this.m_intake = m_intake;
        this.m_arm = m_arm;
        this.m_field = m_field;
        this.m_manager = m_manager;
        this.m_vision = m_vision;
        this.wrist = wrist;

        chooser.addOption("2.5 Cube", new BalanceCubeTwoGamePiece(this));

        chooser.addOption("Cable Auto Balance", new CableAutoBalance(this));

        chooser.addOption("Cable 2 Cube", new CableCubeTwoGamePiece(this));

        chooser.addOption("Center Score Balance", new CenterScoreBalance(this));

        chooser.addOption("Free Auto Balance", new FreeAutoBalance(this));

        chooser.addOption("Free 2 Cube", new FreeCubeTwoGamePiece(this));

        chooser.setDefaultOption("Free Grab Balance", new FreeGrabBalance(this));

        chooser.addOption("Free Grab community", new FreeGrabCommunity(this));

        chooser.addOption("Shoooooot", new Shooooot(this, m_intake, m_vision, wrist, m_arm));

        //chooser.addOption("Two Score One Pickup", new TwoScoreOnePickup(drivebase, m_intake, m_arm, m_field,
        //  m_manager, m_vision, this));

        chooser.addOption("High Cube", new SimpleHighCube(this));

        chooser.addOption("Mid Cone", new SimpleMidCone(this));



        SmartDashboard.putData("Auto Selector", chooser);

    }

    public Command getSelected() {
        return chooser.getSelected();
    }
}