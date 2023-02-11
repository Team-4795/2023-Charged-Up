package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends SequentialCommandGroup{
    public BalanceCommand(DriveSubsystem drive, double speed, double rotation, double duration, double angleThreshold, double errorThreshold){
        addCommands(
            //new DriveCommand(drive, speed, 0, false, duration, angleThreshold),
            new AutoBalanceOld(drive, errorThreshold)
        );
    }
}
