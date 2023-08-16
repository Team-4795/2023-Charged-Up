// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double angleRev = 0.0;
        public double backupAngle = 0.0;
        public double angleRevPerSec = 0.0;
        public double appliedVolts = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmVoltage(double volts) {}
}
