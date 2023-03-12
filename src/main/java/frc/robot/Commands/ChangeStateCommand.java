package frc.robot.Commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateManager;
import frc.robot.StateManager.State;

public class ChangeStateCommand extends CommandBase {
    LiftArm arm;
    EndEffectorIntake intake;
    Wrist wrist;

    boolean run_intake;

    public ChangeStateCommand(boolean run_intake, State state, StateManager manager, LiftArm arm, EndEffectorIntake intake, Wrist wrist) {
        this.arm = arm;
        this.intake = intake;
        this.wrist = wrist;
        
        this.run_intake = run_intake;

        manager.setState(state);

        addRequirements(arm, intake, wrist);
    }

    @Override
    public void execute() {
        wrist.tryExtend(arm.getPosition());
        arm.runAutomatic();

        if (run_intake) {
            intake.intakeAutomatic(arm.getPosition());
        }
    }

    @Override
    public boolean isFinished() {
        return arm.atSetpoint();
    }
}
