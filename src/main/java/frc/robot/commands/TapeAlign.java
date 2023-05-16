// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RotationConstants;
import frc.robot.Constants.TapeAlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;

public class TapeAlign extends CommandBase {
    private Drive drive = Drive.getInstance();
    private Vision vision = Vision.getInstance();
    private boolean isAligned;
    private PIDController rotationPID;
    boolean interrupted = false;

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

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isAligned = false;
        var robotPose = drive.getPose();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double forwardSpeed;
        double x_speed;
        double xSpeed = xspeedSupplier.get();
        double y_speed;
        double ySpeed = yspeedSupplier.get();
        var robotPose2d = drive.getPose();

        if (vision.getTargetAngle() < 2) {
            interrupted = true;
        }

        // if (vision.hasTargets == true) {
        //   double currentHeading = driveSubsystem.getvisionheading();
        //   double rotation = rotationPID.calculate(currentHeading,0);

        //   x_speed = controller.calculate(vision.getTargetAngle(), TapeAlignConstants.kXOffset);
        //   //y_speed = controller.calculate(vision.getTargetAngle(), 0);

        //   driveSubsystem.drive(-x_speed,ySpeed,-rotation,true, true);
        // } else {
        //   double rotation = RotationConstants.kNoTargetSpeed;
        //   if(driveSubsystem.getvisionheading() > 0){
        //     rotation = -RotationConstants.kNoTargetSpeed;
        //   }

        //   driveSubsystem.drive(-xSpeed,ySpeed,-rotation,true,true);
        // }
    }

    @Override
    public void end(boolean interrupted) {}
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isAligned;
    }
}
