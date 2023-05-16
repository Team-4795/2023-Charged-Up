package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager.Gamepiece;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoCommands.Height;

public class Center1CubeBalance extends AutoPath {
    public Command load(AutoCommands autoCommands) {
        return Commands.sequence(
                autoCommands.score(Gamepiece.Cube, Height.High, false),
                autoCommands.outtake(0.2),
                autoCommands.stow(),
                autoCommands.autoBalance(false, true));
    }
}
