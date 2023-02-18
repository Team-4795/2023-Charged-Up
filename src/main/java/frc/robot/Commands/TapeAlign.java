// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.photonvision.PhotonCamera;
//import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class TapeAlign extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Vision vision;
  private boolean isAligned;
  private PIDController rotationPID;
  private final PhotonCamera camera;
  boolean interrupted = false;


  final double P_GAIN = 0.011;
  final double D_GAIN = 0;
  //placeholders
  PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

  public TapeAlign(DriveSubsystem driveSubsystem, Vision vision, PhotonCamera camera) {
    this.driveSubsystem = driveSubsystem;
    this.vision = vision;
    this.camera = camera;

    rotationPID = new PIDController(0.01, 0, 0);
    rotationPID.enableContinuousInput(-180, 180);

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAligned = false;
    var robotPose = driveSubsystem.getPose();
    camera.setPipelineIndex(1); //placeholder
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;
    double x_speed;
    var result = camera.getLatestResult();
    var robotPose2d = driveSubsystem.getPose();

    double currentHeading = driveSubsystem.getvisionheading();
    double rotation = rotationPID.calculate(currentHeading,0);
    x_speed = controller.calculate(vision.getTargetAngle(), 0);
    driveSubsystem.drive(x_speed,-.1, rotation,true, 
    true);

    if (vision.getTargetAngle() < 2) {
      interrupted = true;
    }
 
    }
  
  @Override
  public void end(boolean interrupted) {  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAligned;
  }
}