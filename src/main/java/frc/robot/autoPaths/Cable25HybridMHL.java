package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;

public class Cable25HybridMHL extends SequentialCommandGroup{
    public Cable25HybridMHL(AutoSelector selector){
        PathPlannerTrajectory intakeGP4 = PathPlanner.loadPath("Intake Cable N3 GP4", new PathConstraints(4, 3.5));
        PathPlannerTrajectory scoreGP4 = PathPlanner.loadPath("Score Cable N2 GP4", new PathConstraints(4, 3.5));
        PathPlannerTrajectory intakeGP3 = PathPlanner.loadPath("Intake Cable N2 GP3", new PathConstraints(4, 3.5));
        addCommands( new SequentialCommandGroup(
            selector.autoStartUp(intakeGP4, false),
            selector.score("cone", "mid", false),
            selector.outtake(0.3),
            selector.intakeTrajectory("cube", true, intakeGP4),
            selector.scoreTrajectory("cube", "high", false, scoreGP4),
            selector.outtake(0.3),
            selector.intakeTrajectory("cube", true, intakeGP3)
        ));
    }
}
