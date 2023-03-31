package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;

public class AutoTest extends SequentialCommandGroup{
    public AutoTest(AutoSelector selector){
        PathPlannerTrajectory test = PathPlanner.loadPath("Testing Path", new PathConstraints(2, 2));
        addCommands(new SequentialCommandGroup(
            selector.autoStartUp(test, false),
            selector.scoreTrajectory("cube", "low", false, test),
            selector.outtake(0.5)
        ));
    }
}
