// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;

import javax.swing.plaf.basic.BasicBorders.RolloverButtonBorder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateManager;
import frc.robot.Constants.RollerbarConstants;
import frc.robot.subsystems.*;

public class RollerbarCommand extends CommandBase {
  Rollerbar rollerbar;
  LiftArm arm;
  StateManager manager;
  boolean targetExtend;
  Optional<Double> previousSetpoint;
  boolean returned;

  public RollerbarCommand(Rollerbar rollerbar, LiftArm arm, StateManager manager, boolean extend) {
    this.rollerbar = rollerbar;
    this.arm = arm;
    targetExtend = extend;
    this.manager = manager;
    returned = false;
    addRequirements(rollerbar, arm);
  }

  @Override
  public void initialize() {
    previousSetpoint = manager.getArmSetpoint();
    rollerbar.setTarget(targetExtend);
    if(arm.getPosition() < RollerbarConstants.kArmBoundary){
      arm.setTargetPosition(RollerbarConstants.kArmBoundary + 0.05);
    }
  }

  @Override
  public void execute() {
    arm.runAutomatic();
    rollerbar.tryMove(arm.getPosition());
    if(arm.getPosition() > RollerbarConstants.kArmBoundary + 0.02 && !returned){
      previousSetpoint.ifPresent(arm::setTargetPosition);
      returned = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (rollerbar.isExtended() == targetExtend && arm.atSetpoint());
  }
}
