package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.StateManager.Gamepiece;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoCommands.Height;

public class Cable2Cube extends AutoPath {
    public Command load(AutoCommands autoCommands) {
        PathPlannerTrajectory intakeGP4 = PathPlanner.loadPath("Intake Cable N2 GP4", new PathConstraints(4, 3.5));
        PathPlannerTrajectory scoreGP4 = PathPlanner.loadPath("Score Cable N2 GP4", new PathConstraints(4, 3.5));

        return Commands.sequence(
                autoCommands.autoStartUp(intakeGP4, false),
                autoCommands.score(Gamepiece.Cube, Height.High, false),
                autoCommands.outtake(0.3),
                autoCommands.intakeTrajectory(Gamepiece.Cube, true, intakeGP4),
                autoCommands.scoreTrajectory(Gamepiece.Cube, Height.Mid, false, scoreGP4),
                autoCommands.outtake(0.3),
                autoCommands.stow());
    }
}
