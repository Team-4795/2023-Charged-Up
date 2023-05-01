package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoSelector;
import frc.robot.subsystems.intake.EndEffectorIntake;

public class Cable25HybridMHL extends SequentialCommandGroup {
    public Cable25HybridMHL(AutoSelector selector, EndEffectorIntake intake) {
        PathPlannerTrajectory intakeGP4 = PathPlanner.loadPath("Intake Cable N2 GP4", new PathConstraints(4, 3.5));
        PathPlannerTrajectory scoreGP4 = PathPlanner.loadPath("Score Cable N2 GP4", new PathConstraints(4, 3.5));
        PathPlannerTrajectory intakeGP3 = PathPlanner.loadPath("Intake Cable N2 GP3", new PathConstraints(4, 3.5));
        addCommands(new SequentialCommandGroup(
                selector.autoStartUp(intakeGP4, false),
                selector.score("cube", "high", false),
                new WaitCommand(0.5),
                selector.outtake(0.3),
                selector.intakeTrajectory("cube", true, intakeGP4),
                selector.scoreTrajectory("cube", "mid", false, scoreGP4),
                selector.outtake(0.3),
                selector.intakeTrajectory("cube", true, intakeGP3),
                new InstantCommand(() -> intake.setOverrideStoring(false))));
    }
}
