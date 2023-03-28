package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.AutoSelector;
import frc.robot.StateManager;
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.DriveCommandOld;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class CenterScoreBalance extends SequentialCommandGroup {
    public CenterScoreBalance(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm,
    StateManager m_manager, Vision m_vision, AutoSelector m_autoSelector, Wrist wrist){
        addCommands(
            new SequentialCommandGroup(
                m_autoSelector.score("cube", "high", m_intake, m_manager, m_arm, drivebase, m_vision, wrist),
                m_autoSelector.outtake(m_intake, m_manager, wrist, m_arm, 0.01),
                m_autoSelector.stow(m_intake, m_manager, wrist, m_arm),
                
                new DriveCommandOld(drivebase, AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold,
                                    AutoConstants.checkDuration).withTimeout(AutoConstants.overrideDuration),
                new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold)
            )
        );
    }
}
