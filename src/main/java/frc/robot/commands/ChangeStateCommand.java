// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateManager;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollerbar.Rollerbar;
import frc.robot.subsystems.motorizedWrist.Wrist;

public class ChangeStateCommand extends CommandBase {
    Intake intake;
    Arm arm;
    Wrist wrist;
    Rollerbar rollerbar;
    StateManager manager;
    StateManager.State state;

    boolean end;

    public ChangeStateCommand(
            StateManager.State state,
            Intake intake,
            boolean end,
            Arm arm,
            Wrist wrist,
            Rollerbar rollerbar,
            StateManager manager) {
        this.intake = intake;
        this.arm = arm;
        this.wrist = wrist;
        this.rollerbar = rollerbar;
        this.manager = manager;
        this.state = state;
        this.end = end;

        addRequirements(intake, arm, wrist, rollerbar);
    }

    private void intakeDefault() {
        intake.intake();
    }

    private void rollerbarDefault() {
        if (rollerbar.isExtended()) {
            rollerbar.spin();
        } else {
            rollerbar.stop();
        }
    }

    @Override
    public void initialize() {
        manager.setState(state);
    }

    @Override
    public void execute() {
        intakeDefault();
        rollerbarDefault();
    }

    @Override
    public boolean isFinished() {
        // If `end` is true, end automatically when the arm is at its setpoint
        return arm.atSetpoint() && end;
    }
}
