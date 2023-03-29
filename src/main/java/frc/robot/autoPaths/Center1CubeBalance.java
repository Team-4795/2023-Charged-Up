package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.DriveCommandOld;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.*;

public class Center1CubeBalance extends SequentialCommandGroup {
    public Center1CubeBalance(DriveSubsystem drivebase, AutoSelector m_autoSelector) {
        addCommands(
            new SequentialCommandGroup(
                m_autoSelector.scoreV2("cube", "high", false),
                m_autoSelector.outtake(0.1),
                m_autoSelector.stow(),
                
                new DriveCommandOld(drivebase, AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold,
                                    AutoConstants.checkDuration).withTimeout(AutoConstants.overrideDuration),
                new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold)
            )
        );
    }
}
