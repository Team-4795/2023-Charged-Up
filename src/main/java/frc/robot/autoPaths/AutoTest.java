package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoSelector;

public class AutoTest extends SequentialCommandGroup{
    public AutoTest(AutoSelector selector){
        PathPlannerTrajectory test = PathPlanner.loadPath("Testing Path", new PathConstraints(1, 0.75
        ));
        PathPlannerTrajectory test2 = PathPlanner.loadPath("Testing Path 2", new PathConstraints(1, 0.75));
        addCommands(new SequentialCommandGroup(
            selector.autoStartUp(test, false),
            selector.intakeTrajectory("cube", true, test),
            selector.scoreTrajectory("cube", "low", false, test2),
            selector.outtake(0.5)
        ));
    }
}
