// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

public class LEDCommand extends CommandBase {
  LEDs LEDs;
  double hue;
  Supplier<Double> speed;

  public LEDCommand(LEDs LEDs, Supplier<Double> speed) {
    this.LEDs = LEDs;
    this.speed = speed;

    this.hue = 0;

    addRequirements(LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Hue is 0-180
    this.hue += speed.get() / 20.0;

    for (int i=0; i<LEDs.getLength(); i++) {
      double offset = (double)i / (double)LEDs.getLength();
      double color = (180.0 * (hue + offset));

      color = (color - 1 + 180) % 180;

      if (color < 0.0) color += 180.0;

      LEDs.setHSVIndex(i, (int)color, 255, 255);
    }

    LEDs.setOutput();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
