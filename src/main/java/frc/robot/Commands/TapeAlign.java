// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.TapeAlignConstants;
import frc.robot.Constants.RotationConstants;


public class TapeAlign extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Vision vision;
  private boolean isAligned;
  private PIDController rotationPID;
  boolean interrupted = false;

  private Supplier<Double> xspeedSupplier;
  private Supplier<Double> yspeedSupplier;


  
  PIDController controller = new PIDController(TapeAlignConstants.kP, TapeAlignConstants.kI, TapeAlignConstants.kD);

  public TapeAlign(
    DriveSubsystem driveSubsystem, 
    Vision vision,
    Supplier<Double> i, 
    Supplier<Double> j) {
    this.driveSubsystem = driveSubsystem;
    this.vision = vision;
    this.xspeedSupplier = i;
    this.yspeedSupplier = j;

    rotationPID = new PIDController(RotationConstants.kP, RotationConstants.kI, RotationConstants.kD);
    rotationPID.enableContinuousInput(RotationConstants.kMinimumAngle, RotationConstants.kMaximumAngle);

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAligned = false;
    var robotPose = driveSubsystem.getPose();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;
    double x_speed;
    double xSpeed = xspeedSupplier.get();
    double y_speed; 
    double ySpeed = yspeedSupplier.get();
    var robotPose2d = driveSubsystem.getPose();

    if (vision.getTargetAngle() < 2) {
      interrupted = true;
    }

    if (vision.hasTargets == true) {
      double currentHeading = driveSubsystem.getvisionheading();
      double rotation = rotationPID.calculate(currentHeading,0);

      x_speed = controller.calculate(vision.getTargetAngle(), -4);
      //y_speed = controller.calculate(vision.getTargetAngle(), 0);

      driveSubsystem.drive(-x_speed,ySpeed, rotation,true, true);
    } else {
      // double rotation = .5;
      // driveSubsystem.drive(xSpeed,ySpeed,rotation,true,true);
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