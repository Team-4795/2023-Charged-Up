// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.RotationConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;


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

public class AprilTag extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Vision vision;
  private PIDController rotationPID;
  boolean interrupted = false;


  //PID stuff FIGURE OUT HOW TO DO THIS
  PIDController controller = new PIDController(AprilTagConstants.kP, AprilTagConstants.kI, AprilTagConstants.kD);

  public AprilTag(DriveSubsystem driveSubsystem, Vision vision) {
    this.driveSubsystem = driveSubsystem;
    this.vision = vision;
    addRequirements(driveSubsystem);

    rotationPID = new PIDController(RotationConstants.kP, RotationConstants.kI, RotationConstants.kD);
    rotationPID.enableContinuousInput(RotationConstants.kMinimumAngle, RotationConstants.kMaximumAngle);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var robotPose = driveSubsystem.getPose();
    vision.pipelineIndex(0); //placeholder

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;
    double x_speed;
    var robotPose2d = driveSubsystem.getPose();

    double currentHeading = driveSubsystem.getvisionheading();
    double rotation = rotationPID.calculate(currentHeading,AprilTagConstants.kRotationSetpoint);
    x_speed = controller.calculate(vision.getTargetAngle(), AprilTagConstants.kTranslationSetpoint);
    driveSubsystem.drive(x_speed,-.1, rotation,true, 
    true);

    if (vision.getTargetAngle() < AprilTagConstants.kAngularThreshold) {
      interrupted = true;

    }


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
