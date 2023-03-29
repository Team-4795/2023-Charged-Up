package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;
import frc.robot.subsystems.DriveSubsystem;

public class Cable3CubeLLL extends SequentialCommandGroup{
    public Cable3CubeLLL(DriveSubsystem drivebase, AutoSelector selector){
        PathPlannerTrajectory intakeGP4 = PathPlanner.loadPath("Intake Cable N2 GP4", new PathConstraints(4, 3));
        PathPlannerTrajectory scoreGP4 = PathPlanner.loadPath("Score Cable N1 GP4", new PathConstraints(4, 3));
        PathPlannerTrajectory intakeGP3 = PathPlanner.loadPath("Intake Cable N1 GP3", new PathConstraints(4, 3));
        PathPlannerTrajectory throwGP3 = PathPlanner.loadPath("Throw Cable N3 GP3", new PathConstraints(4, 3));
        addCommands(new SequentialCommandGroup(
            selector.autoStartUp(intakeGP4, false),
            selector.score("cube", "low", false),
            selector.outtake(0.2),
            selector.intakeTrajectory("cube", true, intakeGP4),
            new ParallelDeadlineGroup(
                drivebase.followTrajectoryCommand(scoreGP4),
                selector.score("cube", "low", false)
            ),
            selector.outtake(0.2),
            selector.intakeTrajectory("cube", true, intakeGP3),
            new ParallelDeadlineGroup(
                drivebase.followTrajectoryCommand(throwGP3),
                selector.stow()
            ),
            selector.yeeeeet("cube"),
            selector.stow()
        ));
    }
}
