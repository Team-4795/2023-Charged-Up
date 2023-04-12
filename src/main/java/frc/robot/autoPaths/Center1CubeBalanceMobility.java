package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoSelector;

public class Center1CubeBalanceMobility extends SequentialCommandGroup {
    PathPlannerTrajectory mobility = PathPlanner.loadPath("Mobility Center", new PathConstraints(4, 3));

    public Center1CubeBalanceMobility(AutoSelector m_autoSelector) {
        addCommands(
                new SequentialCommandGroup(
                        m_autoSelector.autoStartUp(mobility, false),
                        m_autoSelector.score("cube", "high", false),
                        m_autoSelector.outtake(0.2),
                        m_autoSelector.stowTrajectory(mobility),
                        new WaitCommand(1),
                        m_autoSelector.autoBalance(false, true)));
    }
}
