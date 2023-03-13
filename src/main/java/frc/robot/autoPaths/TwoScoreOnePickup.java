package frc.robot.autoPaths;

import java.util.Optional;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoSelector;


public class TwoScoreOnePickup extends SequentialCommandGroup {
    public TwoScoreOnePickup(AutoSelector m_autoSelector){
        PathPlannerTrajectory TwoScorePickup = PathPlanner.loadPath("Two GP + Grab 1", new PathConstraints(3, 3));

        addCommands(
            new SequentialCommandGroup(
                m_autoSelector.autoStartUp(TwoScorePickup, true),
                m_autoSelector.scoreV2("cube", "high", Optional.empty()),
                new ParallelCommandGroup(
                    m_autoSelector.followTrajectory(TwoScorePickup),
                    new SequentialCommandGroup(
                        m_autoSelector.intakeV2("cube").withTimeout(0.5),
                        m_autoSelector.stow(),
                        new WaitCommand(1),
                        m_autoSelector.scoreV2("cube", "mid", Optional.empty()),
                        m_autoSelector.intakeV2("cone").withTimeout(0.5),
                        m_autoSelector.stow()))));
    }
}
