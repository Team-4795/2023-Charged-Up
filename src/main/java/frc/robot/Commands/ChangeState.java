package frc.robot.Commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateManager;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Wrist;
import frc.utils.Setpoints;

public class ChangeState extends CommandBase {
    private LiftArm m_arm;
    private EndEffectorIntake m_intake;
    private Wrist m_wrist;
    private Optional<Setpoints> m_setpoints;
    private boolean end = false;

    public ChangeState(StateManager.State state, LiftArm arm, EndEffectorIntake intake, Wrist wrist) {
        m_arm = arm;
        m_intake = intake;
        m_wrist = wrist;
        m_setpoints = state.getSetpoints();

        addRequirements(arm, intake, wrist);
    }

    @Override
    public void initialize() {
        m_setpoints.ifPresent((setpoints) -> m_arm.setPosition(setpoints.arm));
        m_setpoints.ifPresent((setpoints) -> m_wrist.setExtendedTarget(setpoints.wrist));
    }

    @Override
    public void execute() {
        m_intake.intake();

        m_wrist.extended = m_wrist.extendedTarget;

        if (m_arm.getPosition() > ArmConstants.kHighWristLimit || 
            m_arm.getPosition() < ArmConstants.kLowWristLimit)
        {
            m_wrist.extended = false;
        }

        if (m_wrist.extended) {
            m_wrist.extend();
        } else {
            m_wrist.retract();
        }
    }

    @Override
    public void end(boolean interrupt) {
        end = true;
    }

    @Override
    public boolean isFinished() {
        return end || m_setpoints.isEmpty();
    }
}
