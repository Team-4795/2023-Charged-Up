// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class RollerbarCommand extends CommandBase {
  Rollerbar rollerbar;
  LiftArm arm;

  public RollerbarCommand(Rollerbar rollerbar, LiftArm arm) {
    this.rollerbar = rollerbar;
    this.arm = arm;

    addRequirements(rollerbar, arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    rollerbar.tryMove(arm.getPosition());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
