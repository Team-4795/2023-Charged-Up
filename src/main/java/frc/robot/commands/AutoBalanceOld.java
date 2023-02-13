package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceOld extends CommandBase{
    DriveSubsystem drive;
    double elevationAngle;
    double elevationVelocity;
    double errorThreshold;
    double[] output;

    public AutoBalanceOld(DriveSubsystem drive, double errorThreshold){
        this.errorThreshold = errorThreshold;
        this.drive = drive;
        output = new double[2];
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        elevationAngle = drive.getElevationAngle();
        elevationVelocity = drive.getElevationVelocity();
    }

    @Override
    public void execute(){
        elevationAngle = drive.getElevationAngle();
        elevationVelocity = drive.getElevationVelocity();
        output = updateDrive();
        //not sure if Field relative is correct, but whatever
        drive.drive(output[0], output[1], 0, false, true);

        SmartDashboard.putNumber("Angle of Elevation", elevationAngle);
        SmartDashboard.putNumber("X velocity", output[0]);
        SmartDashboard.putNumber("Y velocity", output[1]);
        SmartDashboard.putNumber("Elevation velocity", elevationVelocity);
    }

    private double[] updateDrive() {
        //assuming we drive straight in the x direction for now
        double[] driveValues = new double[2];
        if(elevationAngle > 0){
            driveValues[0] = -(Math.pow(AutoConstants.polyCoeff * (Math.abs(elevationAngle)/AutoConstants.platformMaxAngle), 2)) * AutoConstants.balanceSpeed; 
        } else if(elevationAngle < 0){
            driveValues[0] = (Math.pow(AutoConstants.polyCoeff * (Math.abs(elevationAngle)/AutoConstants.platformMaxAngle), 2)) * AutoConstants.balanceSpeed;
        }
        return driveValues;
    }

    @Override
    public boolean isFinished(){
        return(Math.abs(elevationVelocity) > 0.1 && (elevationAngle/elevationAngle != elevationVelocity/elevationVelocity));
    }

}
