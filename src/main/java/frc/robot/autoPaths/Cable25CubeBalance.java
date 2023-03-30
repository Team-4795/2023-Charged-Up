package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;

public class Cable25CubeBalance extends SequentialCommandGroup{
    public Cable25CubeBalance(AutoSelector selector){
        PathPlannerTrajectory intakeGP4 = PathPlanner.loadPath("Intake Cable N2 GP4", new PathConstraints(4, 3));
        PathPlannerTrajectory scoreMid = PathPlanner.loadPath("Score Cable N2 GP4", new PathConstraints(4, 3));
        PathPlannerTrajectory intakeGP3 = PathPlanner.loadPath("Intake Cable N2 GP3", new PathConstraints(4, 3));
        PathPlannerTrajectory balance = PathPlanner.loadPath("Balance Open GP3", new PathConstraints(4, 3));
        addCommands(new SequentialCommandGroup(
            selector.autoStartUp(intakeGP4, false),
            selector.score("cube", "high", false),
            selector.outtake(0.3),
            selector.intakeTrajectory("cube", true, intakeGP4),
            selector.scoreTrajectory("cube", "mid", false, scoreMid),
            selector.outtake(0.3),
            selector.intakeTrajectory("cube", true, intakeGP3),
            selector.stowTrajectory(balance),
            selector.autoBalance(true, true)
        ));
    }
}
