package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager.Gamepiece;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoCommands.Height;
import frc.robot.subsystems.intake.Intake;

public class Center2CubeBalance extends AutoPath {
    public Command load(AutoCommands autoCommands) {
        PathPlannerTrajectory intakeCenter = PathPlanner.loadPath("Intake Center N2 GP3",
                new PathConstraints(1.5, 2.5));
        return Commands.sequence(
                autoCommands.autoStartUp(intakeCenter, false),
                autoCommands.score(Gamepiece.Cube, Height.High, false),
                autoCommands.outtake(0.2),
                autoCommands.stow(),
                autoCommands.intakeTrajectory(Gamepiece.Cube, true, intakeCenter, 2),
                Commands.parallel(
                        autoCommands.autoBalance(false, true),
                        Commands.sequence(
                                autoCommands.score(Gamepiece.Cube, Height.High, false),
                                Commands.waitSeconds(3),
                                Commands.runOnce(() -> Intake.getInstance()
                                        .setOuttakeSpeed(-1)),
                                autoCommands.outtake(0.3))));
    }
}
