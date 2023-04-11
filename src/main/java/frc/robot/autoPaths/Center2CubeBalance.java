package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;

public class Center2CubeBalance extends SequentialCommandGroup{
    public Center2CubeBalance(AutoSelector selector){
        PathPlannerTrajectory intakeCenter = PathPlanner.loadPath("Intake Center N2 GP3", new PathConstraints(4, 3));
        PathPlannerTrajectory balanceCenter = PathPlanner.loadPath("Balance Open GP3", new PathConstraints(4, 3));
        addCommands(new SequentialCommandGroup(
            selector.autoStartUp(intakeCenter, false),
            selector.score("cube", "high", false),
            selector.outtake(0.2),
            selector.intakeTrajectory("cube", true, intakeCenter),
            selector.scoreTrajectory("cube", "mid", false, balanceCenter),
            selector.autoBalance(true, true).withTimeout(3),
            //add an align?
            selector.outtake(0.5),
            selector.stow(),
            selector.autoBalance(true, false)
        ));
    }
}
