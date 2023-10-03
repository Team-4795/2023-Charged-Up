package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.StateManager.Gamepiece;
import frc.robot.Commands.AutoCommands;

public class AutoTest extends AutoPath {
    public Command load(AutoCommands autoCommands) {
        PathPlannerTrajectory test = PathPlanner.loadPath("Testing Path", new PathConstraints(4, 3.5));

        return Commands.sequence(
            autoCommands.autoStartUp(test, false),
            autoCommands.intakeTrajectory(Gamepiece.Cube, true, test),
            autoCommands.stow()
        );
    }
}
