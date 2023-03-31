// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.StateManager;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RollerbarConstants;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChangeStateCommand extends CommandBase {
  EndEffectorIntake intake;
  LiftArm arm;
  Wrist wrist;
  Rollerbar rollerbar;
  StateManager manager;
  StateManager.State state;

  boolean end;

  public ChangeStateCommand(StateManager.State state, EndEffectorIntake intake, boolean end, LiftArm arm, Wrist wrist,
      Rollerbar rollerbar, StateManager manager) {
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
    intake.intakeFromGamepiece(manager.isStowing());
  }

  private void wristDefault() {
    wrist.extended = wrist.extendedTarget;

    if (arm.getPosition() < ArmConstants.kLowWristLimit) {
      wrist.extended = false;
    }

    if (arm.getPosition() > ArmConstants.kHighWristLimit) {
      wrist.extended = false;
    }

    if (wrist.extended) {
      wrist.extend();
    } else {
      wrist.retract();
    }
  }

  private void armDefault() {
    arm.runAutomatic();
  }

  private void rollerbarDefault() {
    if (rollerbar.isExtended() != rollerbar.getTarget()) {
      if (!rollerbar.safeToMove(arm.setpoint)) {
        arm.setTargetPosition(RollerbarConstants.kArmBoundary + 0.025);
        arm.isTemporary = true;
      }
    } else if (arm.isTemporary) {
      manager.setSetpoints();
      arm.isTemporary = false;
    }

    rollerbar.tryMove(arm.getPosition());

    if (!intake.isStoring() && rollerbar.isExtended() && arm.atSetpoint()) {
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
    wristDefault();
    armDefault();
    rollerbarDefault();
  }

  @Override
  public boolean isFinished() {
    // If `end` is true, end automatically when the arm is at its setpoint
    return arm.atSetpoint() && end;
  }
}
