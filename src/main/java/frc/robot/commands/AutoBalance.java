package frc.robot.Commands;

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
        elevationAngle = drive.getElevationAngleV2();
        elevationVelocity = drive.getElevationVelocityV2();
    }

    @Override
    public void execute(){
        elevationAngle = drive.getElevationAngleV2();
        elevationVelocity = drive.getElevationVelocityV2();
        output = updateDrive();
        drive.setBalanceSpeed(output);
        drive.drive(output, 0.0, 0.0, false, true);
    }

    private double updateDrive() {
        //check angle --> direction relation just in case, should be right tho
        double speed = (Math.pow(AutoConstants.polyCoeff * (Math.abs(elevationAngle)/AutoConstants.platformMaxAngle), 2)) * AutoConstants.balanceSpeed;
        if(elevationAngle > 0){
            speed *= -1;
        }
        return speed;
    }

    private int signOf(double num){
        if(num < 0){
            return -1;
        } else if (num > 0){
            return 1;
        } else {
            return 0;
        }
    }

    @Override
    public void end(boolean interrupted){
        drive.setBalanceSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(elevationVelocity) > errorThreshold && (signOf(elevationAngle) != signOf(elevationVelocity)));
    }

}
