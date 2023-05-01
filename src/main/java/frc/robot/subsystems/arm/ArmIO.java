// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

public interface ArmIO {
  public static class ArmIOInputs {
    public double armAngleRev = 0.0;
    public double armAngleRevPerSec = 0.0;
    public double armAppliedVolts = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmVoltage(double volts) {}
}
