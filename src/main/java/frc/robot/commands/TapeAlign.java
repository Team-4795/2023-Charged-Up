// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RotationConstants;
import frc.robot.Constants.TapeAlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;

public class TapeAlign extends CommandBase {
    private Drive drive = Drive.getInstance();
    private Vision vision = Vision.getInstance();
    private PIDController rotationPID;

    private Supplier<Double> xspeedSupplier;
    private Supplier<Double> yspeedSupplier;

    PIDController controller = new PIDController(TapeAlignConstants.kP, TapeAlignConstants.kI, TapeAlignConstants.kD);

    public TapeAlign(Supplier<Double> i, Supplier<Double> j) {
        this.xspeedSupplier = i;
        this.yspeedSupplier = j;

        rotationPID = new PIDController(RotationConstants.kP, RotationConstants.kI, RotationConstants.kD);
        rotationPID.enableContinuousInput(RotationConstants.kMinimumAngle, RotationConstants.kMaximumAngle);

        addRequirements(drive);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x_speed;
        double xSpeed = xspeedSupplier.get();
        double ySpeed = yspeedSupplier.get();
        
        if(vision.hasTargets()){
            double currentHeading = drive.getVisionHeading();
            double rotation = rotationPID.calculate(currentHeading, 0);

            x_speed = controller.calculate(vision.getAngleX(), TapeAlignConstants.kXOffset);

            drive.runVelocity(new ChassisSpeeds(
                x_speed * DriveConstants.kMaxSpeedMetersPerSecond, 
                ySpeed * DriveConstants.kMaxSpeedMetersPerSecond, 
                -rotation * DriveConstants.kMaxAngularSpeed));
        } else {
            double rotation = RotationConstants.kNoTargetSpeed;
            if(drive.getVisionHeading() > 0){
                rotation *= -1;
            }

            drive.runVelocity(new ChassisSpeeds(
                xSpeed * DriveConstants.kMaxSpeedMetersPerSecond, 
                ySpeed * DriveConstants.kMaxSpeedMetersPerSecond, 
                -rotation * DriveConstants.kMaxAngularSpeed));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}