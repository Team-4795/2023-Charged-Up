package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AutoCommands;

public abstract class AutoPath {
    public abstract Command load(AutoCommands autoCommands);
}
