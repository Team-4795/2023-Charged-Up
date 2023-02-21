// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class HiLetGo {
  private final DigitalInput sensor;

  public HiLetGo(int port) {
    sensor = new DigitalInput(port);
  }

  public boolean isBroken() {
    return !sensor.get();
}
}