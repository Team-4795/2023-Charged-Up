// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.TapeAlignConstants;
import frc.robot.Constants.RotationConstants;


public class TapeAlign extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Vision vision;
  private PIDController rotationPID;

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

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x_speed;
    double xSpeed = xspeedSupplier.get();
    double ySpeed = yspeedSupplier.get();

    if (vision.hasTargets()) {
      double currentHeading = driveSubsystem.getvisionheading();
      double rotation = rotationPID.calculate(currentHeading,0);

      x_speed = controller.calculate(vision.getAngleX(), TapeAlignConstants.kXOffset);

      driveSubsystem.drive(x_speed, ySpeed, rotation,true, true);
    } else {
      double rotation = RotationConstants.kNoTargetSpeed;
      if(driveSubsystem.getvisionheading() > 0){
        rotation = -RotationConstants.kNoTargetSpeed;
      }

      driveSubsystem.drive(xSpeed, ySpeed, -rotation,true,true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}