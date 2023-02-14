package frc.robot.commands;

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
    
    double output;

    public AutoBalance(DriveSubsystem drive, double errorThreshold){
        this.drive = drive;
        this.errorThreshold = errorThreshold;
        output = 0;

        addRequirements(drive);
    }


    @Override
    public void initialize(){
        elevationAngle = drive.getElevationAngle();
        elevationVelocity = drive.getElevationVelocity();
    }

    @Override
    public void execute(){
        elevationAngle = drive.getElevationAngleV2();
        elevationVelocity = drive.getElevationVelocityV2();
        output = updateDrive();
        drive.drive(output, 0, 0, false, true);

        SmartDashboard.putNumber("Angle of Elevation", elevationAngle);
        SmartDashboard.putNumber("Speed", output);
        SmartDashboard.putNumber("Elevation velocity", elevationVelocity);
    }

    private double updateDrive() {
        //check angle --> direction relation just in case, should be right tho
        double speed = (Math.pow(AutoConstants.polyCoeff * (Math.abs(elevationAngle)/AutoConstants.platformMaxAngle), 2)) * AutoConstants.balanceSpeed;
        if(elevationAngle > 0){
            speed *= -1;
        }
        return speed;
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(elevationVelocity) > 0.2 && (elevationAngle/elevationAngle != elevationVelocity/elevationVelocity));
    }

}
