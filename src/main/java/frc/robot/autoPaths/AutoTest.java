package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoSelector;

public class AutoTest extends SequentialCommandGroup{
    public AutoTest(AutoSelector selector){
        PathPlannerTrajectory test = PathPlanner.loadPath("Testing Path", new PathConstraints(1, 1));
        PathPlannerTrajectory test2 = PathPlanner.loadPath("Testing Path 2", new PathConstraints(1, 1));
        addCommands(new SequentialCommandGroup(
            selector.autoStartUp(test, false),
            selector.stowTrajectory(test),
            new WaitCommand(1),
            selector.stowTrajectory(test2)
        ));
    }
}
