package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoSelector;

public class AutoTest extends SequentialCommandGroup{
    public AutoTest(AutoSelector selector){
        PathPlannerTrajectory test = PathPlanner.loadPath("Testing Path", new PathConstraints(4, 3.5
        ));
        PathPlannerTrajectory test2 = PathPlanner.loadPath("Testing Path 2", new PathConstraints(4, 3.5));
        PathPlannerTrajectory test3 = PathPlanner.loadPath("Testing Path 3", new PathConstraints(4, 3.5));
        PathPlannerTrajectory test4 = PathPlanner.loadPath("Testing Path 4", new PathConstraints(4, 3.5));
        PathPlannerTrajectory test5 = PathPlanner.loadPath("Testing Path 5", new PathConstraints(4, 3.5));
        addCommands(new SequentialCommandGroup(
            selector.autoStartUp(test3, false),
            selector.intakeTrajectory("cube", true, test3),
            selector.stow()
        ));
    }
}
