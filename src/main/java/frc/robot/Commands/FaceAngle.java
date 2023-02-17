// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.Constants;
import frc.robot.Constants.RotationConstants;


public class FaceAngle extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private Supplier<Double> xSupplier;
  private Supplier<Double> ySupplier;
  private Supplier<Double> setpointSupplier;
  private PIDController rotationPID;
  private boolean isAligned;

  public FaceAngle(

    DriveSubsystem driveSubsystem,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    double setpoint)

    {
    rotationPID = new PIDController(RotationConstants.kP, RotationConstants.kI, RotationConstants.kD);
    rotationPID.enableContinuousInput(RotationConstants.kMinimumAngle, RotationConstants.kMaximumAngle);
    addRequirements(driveSubsystem);
  }


  @Override
  public void initialize() {
    isAligned = false;
  }

  @Override
  public void execute() {
    double xspeed = xSupplier.get();
    double yspeed = ySupplier.get();
    double setpoint = setpointSupplier.get();
    double currentHeading = driveSubsystem.getvisionheading();
    double rotation = rotationPID.calculate(currentHeading,setpoint);
    driveSubsystem.drive(xspeed, yspeed, rotation,true, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isAligned;
  }
}