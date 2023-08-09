// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveWithJoysticks extends CommandBase {
    private static final double deadband = 0.1;

    private Drive drive;
    private final Supplier<Double> leftXSupplier;
    private final Supplier<Double> leftYSupplier;
    private final Supplier<Double> rightYSupplier;
    private final Supplier<Boolean> robotRelativeOverride;

    private final double linearSpeedLimit = 1.0;
    private final double angularSpeedLimit = 1.0;

    /** Creates a new DriveWithJoysticks. */
    public DriveWithJoysticks(
            Drive drive,
            Supplier<Double> leftXSupplier,
            Supplier<Double> leftYSupplier,
            Supplier<Double> rightYSupplier,
            Supplier<Boolean> robotRelativeOverride) {

        this.drive = drive;
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.rightYSupplier = rightYSupplier;
        this.robotRelativeOverride = robotRelativeOverride;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Get values from double suppliers
        double leftX = leftXSupplier.get();
        double leftY = leftYSupplier.get();
        double rightY = rightYSupplier.get();

        // Get direction and magnitude of linear axes
        double linearMagnitude = Math.hypot(leftX, leftY);
        Rotation2d linearDirection = new Rotation2d(leftX, leftY);

        // Apply deadband
        linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);
        rightY = MathUtil.applyDeadband(rightY, deadband);

        // Apply squaring
        linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
        rightY = Math.copySign(rightY * rightY, rightY);

        // Apply speed limits
        linearMagnitude *= linearSpeedLimit;
        rightY *= angularSpeedLimit;

        // Calcaulate new linear components
        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(new Translation2d(linearMagnitude, 0.0), new Rotation2d()))
                .getTranslation();

        // Convert to meters per second
        ChassisSpeeds speeds = new ChassisSpeeds(
                linearVelocity.getX() * DriveConstants.kMaxSpeedMetersPerSecond,
                linearVelocity.getY() * DriveConstants.kMaxSpeedMetersPerSecond,
                rightY * DriveConstants.kMaxAngularSpeed);

        // Convert from field relative
        if (!robotRelativeOverride.get()) {
            var driveRotation = drive.getRotation();
            if (DriverStation.getAlliance() == Alliance.Red) {
                driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
            }
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, driveRotation);
        }

        // Send to drive
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
