package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;

/*
 * May need to interpolate elevation velocity between x and y axis
 */
public class AutoBalance extends CommandBase{
    DriveSubsystem drive;
    double errorThreshold;
    
    double elevationAngle;
    double elevationVelocity;
    Rotation2d heading;
    
    double[] output;

    public AutoBalance(DriveSubsystem drive, double errorThreshold){
        this.drive = drive;
        this.errorThreshold = errorThreshold;
        output = new double[3];

        addRequirements(drive);
    }


    @Override
    public void initialize(){
        elevationAngle = drive.getElevationAngle();
        elevationVelocity = drive.getElevationVelocity();
        heading = drive.getHeading();
    }

    @Override
    public void execute(){
        elevationAngle = drive.getElevationAngle();
        elevationVelocity = drive.getElevationVelocity();
        heading = drive.getHeading();
        output = updateDrive();
        drive.drive(output[0], output[1], 0, false, true);

        SmartDashboard.putNumber("Angle of Elevation", elevationAngle);
        SmartDashboard.putNumber("X velocity", output[0]);
        SmartDashboard.putNumber("Y velocity", output[1]);
        SmartDashboard.putNumber("Elevation velocity", elevationVelocity);
        SmartDashboard.putNumber("Heading", heading.getDegrees());
    }

    private double[] updateDrive() {
        double[] driveValues = new double[2];
        double speed = (Math.pow(AutoConstants.polyCoeff * (Math.abs(elevationAngle)/AutoConstants.platformMaxAngle), 2)) * AutoConstants.balanceSpeed;
        if(elevationAngle < 0){
            speed *= -1;
        }
        driveValues[0] = speed * heading.getCos();
        driveValues[1] = speed * heading.getSin();
        return driveValues;
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(elevationVelocity) > 0.1 && (elevationAngle/elevationAngle != elevationVelocity/elevationVelocity));
    }

}
