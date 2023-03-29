package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;
import frc.robot.subsystems.DriveSubsystem;

public class Center15CubeBalance extends SequentialCommandGroup{
    public Center15CubeBalance(DriveSubsystem drivebase, AutoSelector selector){
        PathPlannerTrajectory intakeCenter = PathPlanner.loadPath("Intake Center N2 GP2", new PathConstraints(4, 3));
        PathPlannerTrajectory balanceCenter = PathPlanner.loadPath("Balance Center Open GP2", new PathConstraints(4, 3));

        addCommands(new SequentialCommandGroup(
            selector.autoStartUp(intakeCenter, false),
            selector.score("cube", "high", false),
            selector.outtake(0.2),
            selector.intakeTrajectory("cube", true, intakeCenter),
            drivebase.followTrajectoryCommand(balanceCenter),
            selector.autoBalance(false, false)
        ));
    }
}