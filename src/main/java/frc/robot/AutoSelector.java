package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

    public Command scoreV2(String gamepiece, String setpoint, Optional<PathPlannerTrajectory> traj) {
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
                // new TapeAlign(drivebase, m_vision, () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed).withTimeout(1),
                new ConditionalCommand(drivebase.followTrajectoryCommand(traj.get()), Commands.none(), () -> !traj.isEmpty()),
                new ChangeStateCommand(true, newState, m_manager, m_arm, m_intake, wrist).withTimeout(1)
            ),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(0.5)
        );
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
            new ChangeStateCommand(true, State.LowPickup, m_manager, m_arm, m_intake, wrist).withTimeout(0.7),
            new RunCommand(() -> m_intake.runIntake(), m_intake)
                .withTimeout(maxDuration)
        );
    }

    public Command stow() {
        return new ChangeStateCommand(true, State.StowInFrame, m_manager, m_arm, m_intake, wrist).withTimeout(1);
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

        chooser.addOption("Free Grab Balance", new FreeGrabBalance(this));

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