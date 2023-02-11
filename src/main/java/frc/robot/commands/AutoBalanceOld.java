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
        output = new double[3];
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
        drive.drive(output[0], output[1], output[2], false, true);
        SmartDashboard.putNumber("Angle of Elevation", elevationAngle);
        SmartDashboard.putNumber("X velocity", output[0]);
        SmartDashboard.putNumber("Y velocity", output[1]);
        SmartDashboard.putNumber("Elevation velocity", elevationVelocity);
    }

    private double[] updateDrive() {
        //assuming we drive straight in the x direction for now; negative signs could be flipped
        double[] driveValues = new double[3];
        if(elevationAngle > 0){
            driveValues[0] = -(power(1.5 * (Math.abs(elevationAngle)/AutoConstants.platformMaxAngle), 2)) * AutoConstants.balanceSpeed; 
        } else if(elevationAngle < 0){
            driveValues[0] = (power(1.5 * (Math.abs(elevationAngle)/AutoConstants.platformMaxAngle), 2)) * AutoConstants.balanceSpeed;
        }
        return driveValues;
    }

    private static double power(double a, int b){
        double sum = a;
        for(int i = 1; i < b; i++){
            sum *= a;
        }
        return a;
    }

    @Override
    public boolean isFinished(){
        //return (Math.abs(elevationAngle) < errorThreshold);
        return(Math.abs(elevationVelocity) > 0.1 && (elevationAngle/elevationAngle != elevationVelocity/elevationVelocity));
    }

}
