package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager.Gamepiece;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Commands.AutoCommands;
import frc.robot.Commands.AutoCommands.Height;

public class Free3HybridMHM extends AutoPath {
    public Command load(AutoCommands autoCommands) {
        PathPlannerTrajectory intakeGP1 = PathPlanner.loadPath("Intake Free N1 GP1", new PathConstraints(3.5, 3.5));
        PathPlannerTrajectory scoreGP1 = PathPlanner.loadPath("Score Free N2 GP1", new PathConstraints(3.0, 3.5));
        PathPlannerTrajectory intakeGP2 = PathPlanner.loadPath("Intake Free N2 GP2", new PathConstraints(3.5, 3.5));
        PathPlannerTrajectory scoreGP2 = PathPlanner.loadPath("Score Free N2 GP2", new PathConstraints(3.5, 3.5));
        return Commands.sequence(new SequentialCommandGroup(
                autoCommands.autoStartUp(intakeGP1, false),
                autoCommands.score(Gamepiece.Cone, Height.Mid, false),
                autoCommands.outtake(0.2),
                autoCommands.intakeTrajectory(Gamepiece.Cube, true, intakeGP1),
                autoCommands.scoreTrajectory(Gamepiece.Cube, Height.High, false, scoreGP1),
                Commands.waitSeconds(0.2),
                autoCommands.outtake(0.2),
                autoCommands.intakeTrajectory(Gamepiece.Cube, true, intakeGP2),
                autoCommands.scoreTrajectory(Gamepiece.Cube, Height.Mid, false, scoreGP2),
                autoCommands.outtake(0.3),
                autoCommands.intakeTrajectory(Gamepiece.Cube, true, intakeGP2)).finallyDo((end) -> {
                    Intake.getInstance().setOverrideStoring(false);
                }));
                
    }
}
