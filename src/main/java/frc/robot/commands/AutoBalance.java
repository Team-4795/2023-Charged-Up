package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;


public class AutoBalance extends CommandBase{
    DriveSubsystem drive;
    double elevationAngle;
    double errorThreshold;
    double[] output;

    public AutoBalance(double errorThreshold){
        this.errorThreshold = errorThreshold;
        output = new double[3];
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        elevationAngle = drive.getElevationAngle();
    }

    @Override
    public void execute(){
        elevationAngle = drive.getElevationAngle();
        output = updateDrive();
        //not sure if Field relative is correct, but whatever
        drive.drive(output[0], output[1], output[2], true);
        SmartDashboard.putNumber("Angle of Elevation", elevationAngle);
        SmartDashboard.putNumber("X velocity", output[0]);
        SmartDashboard.putNumber("Y velocity", output[1]);
    }

    private double[] updateDrive() {
        //assuming we drive straight in the x direction for now; negative signs could be flipped
        double[] driveValues = new double[3];
        if(elevationAngle > 0){
            driveValues[0] = (1 + elevationAngle / AutoConstants.platformMaxAngle) * AutoConstants.balanceSpeed; 
        } else if(elevationAngle < 0){
            driveValues[0] = -(1 + elevationAngle / AutoConstants.platformMaxAngle) * AutoConstants.balanceSpeed;
        }
        return driveValues;
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(elevationAngle) < errorThreshold);
    }

}
