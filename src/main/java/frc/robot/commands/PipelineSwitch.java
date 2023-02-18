// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.vision.VisionPipeline;
import frc.robot.subsystems.Vision;
import frc.robot.commands.Align;
import frc.robot.commands.TapeAlign;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonCamera;
import frc.robot.Constants;
import org.photonvision.common.hardware.VisionLEDMode;

public class PipelineSwitch extends CommandBase {
  private final PhotonCamera camera;
  private final Vision vision;

  public void SwitchToAlign() {
    camera.setPipelineIndex(1);
    disableLED(VisionLEDMode.kOff);
  }

  public void SwitchToTape() {
    camera.setPipelineIndex(0);
    enableLED(VisionLEDMode.kOn);
  }
  
  /** Creates a new PipelineSwitch. */
  public PipelineSwitch(PhotonCamera camera, Vision vision) {
    this.camera = camera;
    this.vision = vision;

    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setPipelineIndex(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
