// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.StateManager;
// import frc.robot.commands.Align;
// import frc.robot.commands.TapeAlign;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.PhotonCamera;
// import frc.robot.Constants;
// import org.photonvision.common.hardware.VisionLEDMode;
// import frc.robot.Constants.VisionConstants;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.vision.VisionPipeline;


public class PipelineSwitch extends CommandBase {
  private final Vision vision;
  private final StateManager state;
  public int pipelineIndex;

  /** Creates a new PipelineSwitch. */
  public PipelineSwitch(Vision vision, StateManager state) {
    this.vision = vision;
    this.state = state;

    addRequirements(vision);
    vision.pipelineIndex(1);
    //by default what do we want the pipeline to be?
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (pipelineIndex == 1) {
      vision.switchToTag();
    }
    else {
      vision.switchToTape();
    }
  }

  // public boolean targetLED() {
  //   return vision.isTargeting;
  // }

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