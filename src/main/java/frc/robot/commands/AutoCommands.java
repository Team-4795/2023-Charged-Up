package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateManager;
import frc.robot.StateManager.Gamepiece;
import frc.robot.StateManager.State;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollerbar.Rollerbar;
import frc.robot.subsystems.WristV2.Wrist;
import java.util.HashMap;
import java.util.Map;

public class AutoCommands {
    private HashMap<String, PathPlannerTrajectory> paths = new HashMap<>();

    private Swerve drivebase = Swerve.getInstance();
    private Arm arm = Arm.getInstance();
    private Rollerbar rollerbar = Rollerbar.getInstance();
    private Intake intake = Intake.getInstance();
    private Wrist wrist = Wrist.getInstance();
    private StateManager manager = StateManager.getInstance();

    public enum Height {
        Low,
        Mid,
        High;
    }

    public AutoCommands() {}

    public PathPlannerTrajectory loadPath(String pathName, PathConstraints constraints) {
        return paths.computeIfAbsent(pathName, k -> PathPlanner.loadPath(k, constraints));
    }

    public Command score(Gamepiece gamepiece, Height height, boolean backwards) {
        Command setGamepiece;

        switch (gamepiece) {
            case Cube:
                setGamepiece = Commands.runOnce(manager::pickCube);
                break;
            case Cone:
                setGamepiece = Commands.runOnce(manager::pickCone);
                break;
            default:
                throw new AssertionError();
        }

        State state;

        if (backwards) {
            switch (height) {
                case Low:
                    state = State.BackwardsLowPickup;
                    break;
                case Mid:
                    state = State.BackwardsMidScore;
                    break;
                case High:
                    state = State.BackwardsHighScore;
                    break;
                default:
                    throw new AssertionError();
            }
        } else {
            switch (height) {
                case Low:
                    state = State.LowPickup;
                    break;
                case Mid:
                    state = State.MidScore;
                    break;
                case High:
                    state = State.HighScore;
                    break;
                default:
                    throw new AssertionError();
            }
        }

        Command setOverride = Commands.runOnce(() -> intake.setOverrideStoring(true));

        return Commands.sequence(
                setGamepiece, setOverride, new ChangeStateCommand(state, intake, true, arm, wrist, rollerbar, manager));
    }

    public Command scoreTrajectory(Gamepiece gamepiece, Height height, boolean backwards, PathPlannerTrajectory traj) {
        return Commands.parallel(
                drivebase.followTrajectoryCommand(traj),
                Commands.sequence(
                        Commands.waitSeconds(AutoConstants.kIntakeDelay), score(gamepiece, height, backwards)));
    }

    public Command intakeTrajectory(Gamepiece gamepiece, boolean backwards, PathPlannerTrajectory traj) {
        return intakeTrajectory(gamepiece, backwards, traj, 0.0);
    }

    public Command intakeTrajectory(Gamepiece gamepiece, boolean backwards, PathPlannerTrajectory traj, double time) {
        Command setGamepiece;

        switch (gamepiece) {
            case Cube:
                setGamepiece = Commands.runOnce(manager::pickCube);
                break;
            case Cone:
                setGamepiece = Commands.runOnce(manager::pickCone);
                break;
            default:
                throw new AssertionError();
        }

        StateManager.State state;

        if (backwards) {
            state = State.BackwardsLowPickupAuto;
        } else {
            state = State.LowPickup;
        }

        return Commands.sequence(
                setGamepiece,
                Commands.deadline(
                                drivebase
                                        .followTrajectoryCommand(traj)
                                        .andThen(Commands.waitSeconds(AutoConstants.kIntakeWaitTime)),
                                Commands.sequence(
                                        Commands.run(intake::outtake, intake).withTimeout(AutoConstants.kOuttakeDelay),
                                        Commands.waitSeconds(time),
                                        new ChangeStateCommand(state, intake, false, arm, wrist, rollerbar, manager)))
                        .finallyDo((x) -> intake.setOverrideStoring(true)));
    }

    public Command stow() {
        return new ChangeStateCommand(State.StowInFrame, intake, true, arm, wrist, rollerbar, manager);
    }

    public Command stowTrajectory(PathPlannerTrajectory traj) {
        return stow().alongWith(drivebase.followTrajectoryCommand(traj));
    }

    public Command outtake(double time) {
        Map<Object, Command> outtake = Map.ofEntries(
                Map.entry(1, Commands.run(intake::outtake, intake).withTimeout(time)),
                Map.entry(
                        2,
                        Commands.sequence(
                                Commands.runOnce(wrist::extend),
                                Commands.waitSeconds(0.3),
                                Commands.run(intake::outtake, intake).withTimeout(time))),
                Map.entry(
                        3,
                        Commands.sequence(
                                Commands.runOnce(wrist::retract),
                                Commands.waitSeconds(IntakeConstants.kFlickTime),
                                Commands.run(intake::outtake, intake).withTimeout(time))));

        return Commands.select(outtake, () -> {
                    if (manager.getState() == State.MidScore && manager.getGamepiece() == Gamepiece.Cone) {
                        return 2;
                    } else if (manager.getState() == State.BackwardsHighScore
                            && manager.getGamepiece() == Gamepiece.Cube) {
                        return 3;
                    } else {
                        return 1;
                    }
                })
                .andThen(Commands.runOnce(() -> intake.setOverrideStoring(false)));
    }

    public Command autoStartUp(PathPlannerTrajectory traj, boolean flip) {
        return Commands.sequence(Commands.runOnce(intake::resetStoring), drivebase.AutoStartUp(traj, flip));
    }

    public Command autoBalance(boolean backward, boolean withDriveup) {
        int direction = 1;
        if (backward) {
            direction = -1;
        }

        if (withDriveup) {
            return Commands.sequence(
                    new DriveCommand(
                            direction * AutoConstants.driveBalanceSpeed,
                            AutoConstants.driveAngleThreshold,
                            AutoConstants.checkDuration),
                    new AutoBalance(AutoConstants.angularVelocityErrorThreshold));
        } else {
            return new AutoBalance(AutoConstants.angularVelocityErrorThreshold);
        }
    }
}
