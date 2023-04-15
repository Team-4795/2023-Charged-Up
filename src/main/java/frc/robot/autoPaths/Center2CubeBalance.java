package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutoSelector;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;

public class Center2CubeBalance extends SequentialCommandGroup {
    public Center2CubeBalance(DriveSubsystem drivebase, AutoSelector selector, EndEffectorIntake intake) {
        PathPlannerTrajectory intakeCenter = PathPlanner.loadPath("Intake Center N2 GP3", new PathConstraints(1.5, 2.5));
        PathPlannerTrajectory balanceCenter = PathPlanner.loadPath("Balance Open GP3", new PathConstraints(1.5, 2.5));
        addCommands(new SequentialCommandGroup(
                selector.autoStartUp(intakeCenter, false),
                selector.score("cube", "high", false),
                selector.outtake(0.2),
                selector.stow(),
                selector.intakeTrajectory("cube", true, intakeCenter, 2),
                selector.scoreTrajectory("cube", "high", false, balanceCenter),
                new ParallelCommandGroup(
                        selector.autoBalance(false, false),
                        new SequentialCommandGroup(
                                new WaitCommand(3),
                                new InstantCommand(() -> intake.setOuttakeSpeed(-1)),
                                selector.outtake(0.3)
                                ))));
    }
}
