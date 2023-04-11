package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;

public class Free3HybridMHM extends SequentialCommandGroup{
    public Free3HybridMHM(AutoSelector selector){
        PathPlannerTrajectory intakeGP1 = PathPlanner.loadPath("Intake Free N1 GP1", new PathConstraints(4, 3.5));
        PathPlannerTrajectory scoreGP1 = PathPlanner.loadPath("Score Free N2 GP1", new PathConstraints(4, 3.5));
        PathPlannerTrajectory intakeGP2 = PathPlanner.loadPath("Intake Free N2 GP2", new PathConstraints(4, 3.5));
        PathPlannerTrajectory scoreGP2 = PathPlanner.loadPath("Score Free N2 GP2", new PathConstraints(4, 3.5));
        addCommands(new SequentialCommandGroup(
            selector.autoStartUp(intakeGP1, false),
            selector.score("cone", "mid", false),
            selector.outtake(0.2),
            selector.intakeTrajectory("cube", true, intakeGP1),
            selector.scoreTrajectory("cube", "high", false, scoreGP1),
            selector.outtake(0.2),
            selector.intakeTrajectory("cube", true, intakeGP2),
            selector.scoreTrajectory("cube", "mid", false, scoreGP2),
            selector.outtake(0.3),
            selector.stow()
        ));
    }
}
