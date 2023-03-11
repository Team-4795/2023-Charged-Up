package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoSelector;
import frc.robot.StateManager;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;

public class TwoScoreOnePickup extends SequentialCommandGroup {
    public TwoScoreOnePickup(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
    StateManager m_manager, Vision m_vision, AutoSelector m_autoSelector){
        PathPlannerTrajectory TwoScorePickup = PathPlanner.loadPath("Two GP + Grab 1", new PathConstraints(3, 3));

        addCommands(
            new SequentialCommandGroup(
                drivebase.AutoStartUp(TwoScorePickup),
                m_autoSelector.score("cube", "high", m_intake, m_manager, m_arm, drivebase, m_vision),
                new ParallelCommandGroup(
                    drivebase.followTrajectoryCommand(TwoScorePickup),
                    new SequentialCommandGroup(
                        m_autoSelector.intake("cube", m_intake, m_manager, m_arm),
                        new WaitCommand(2.5),
                        m_autoSelector.stow(m_intake, m_manager, m_arm),
                        new WaitCommand(1),
                        m_autoSelector.scoreV2("cube", "mid", m_intake, m_manager, m_arm, drivebase, m_vision),
                        m_autoSelector.intake("cone", m_intake, m_manager, m_arm),
                        new WaitCommand(3.5),
                        m_autoSelector.stow(m_intake, m_manager, m_arm)
                    )
                )

            )
        );
    }
    
}