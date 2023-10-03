package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager.Gamepiece;
import frc.robot.Commands.AutoCommands;
import frc.robot.Commands.AutoCommands.Height;

public class Center1CubeBalanceMobility extends AutoPath {
    public Command load(AutoCommands autoCommands) {
        PathPlannerTrajectory mobility = PathPlanner.loadPath("Mobility Center", new PathConstraints(1.5, 3));

        return Commands.sequence(
                autoCommands.autoStartUp(mobility, false),
                autoCommands.score(Gamepiece.Cube, Height.High, false),
                autoCommands.outtake(0.2),
                autoCommands.stowTrajectory(mobility),
                Commands.waitSeconds(0.4),
                autoCommands.autoBalance(false, true));
    }
}
