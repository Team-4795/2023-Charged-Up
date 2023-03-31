package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;

public class Free25CubeBalance extends SequentialCommandGroup{
    public Free25CubeBalance(AutoSelector selector){
        PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Intake Free N2 GP1",
        new PathConstraints(4, 3));
        PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Score Free N2 GP1",
        new PathConstraints(4, 3));
        PathPlannerTrajectory CubeThreeGamePiece1 = PathPlanner.loadPath("Intake Free N2 GP2",
        new PathConstraints(4, 3));   
        PathPlannerTrajectory balance = PathPlanner.loadPath("Balance Open GP2",
        new PathConstraints(4, 3));   

    // Add option of Vision based two game peice split into parts with commands Cube
    addCommands(
        new SequentialCommandGroup(
            selector.autoStartUp(CubeTwoGamePiece1, false),
            selector.score("cube", "high", false),
            selector.outtake(0.3),

            selector.intakeTrajectory("cube", true, CubeTwoGamePiece1),
            selector.scoreTrajectory("cube", "mid", false, CubeTwoGamePiece2),

            selector.outtake(0.3),
            selector.intakeTrajectory("cube", true, CubeThreeGamePiece1),
            selector.stowTrajectory(balance),
            selector.autoBalance(true, true)
        ));
    }
}
