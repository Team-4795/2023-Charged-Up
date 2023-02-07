// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.lang.annotation.Target;

public class TapeAlign extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Vision vision;
  private boolean isAligned;
  private long alignStart; 
  private PIDController rotationPID;
  private final PhotonCamera camera;

  final double P_GAIN = 0.1;
  final double D_GAIN = 0.1;
  //placeholders
  PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

  public TapeAlign(DriveSubsystem driveSubsystem, Vision vision, PhotonCamera camera) {
    this.driveSubsystem = driveSubsystem;
    this.vision = vision;
    this.camera = camera;

    rotationPID = new PIDController(0.1, 0, 0);
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAligned = false;
    alignStart = System.currentTimeMillis();

    var robotPose = driveSubsystem.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;
    double x_speed;


    var result = camera.getLatestResult();
    var robotPose2d = driveSubsystem.getPose();
    if (result.hasTargets()) {
      double currentHeading = driveSubsystem.getHeading();
      double error = 0 - currentHeading;
      double rotation = rotationPID.calculate(error);
      driveSubsystem.drive(0, 0, rotation, true);
      x_speed = -rotationPID.calculate(result.getBestTarget().getYaw(), 0);
      driveSubsystem.drive(0,x_speed,0,true);
    } else {
      driveSubsystem.drive(0,0,0,true);
    }
  }
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
