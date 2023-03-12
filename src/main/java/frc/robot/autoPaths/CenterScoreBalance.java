package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;


public class CenterScoreBalance extends SequentialCommandGroup {
    public CenterScoreBalance(AutoSelector m_autoSelector) {
        addCommands(
            new SequentialCommandGroup(
                m_autoSelector.scoreV2("cube", "high"),
                m_autoSelector.stow(),
                m_autoSelector.autoBalanceOld()
            )
        );
    }
}