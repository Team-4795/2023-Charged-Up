package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;

public class Center1CubeBalance extends SequentialCommandGroup {
    public Center1CubeBalance(AutoSelector m_autoSelector) {
        addCommands(
            new SequentialCommandGroup(
                m_autoSelector.score("cube", "high", false),
                m_autoSelector.outtake(0.1),
                m_autoSelector.stow(),

                m_autoSelector.autoBalance(true, true)
            )
        );
    }
}
