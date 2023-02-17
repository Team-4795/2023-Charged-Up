package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase{
    DriveSubsystem drive;
    double elevationAngle;
    double elevationVelocity;
    double errorThreshold;
    double output;

    public AutoBalance(DriveSubsystem drive, double errorThreshold){
        this.errorThreshold = errorThreshold;
        this.drive = drive;
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
        elevationAngle = drive.getElevationAngle();
        elevationVelocity = drive.getElevationVelocity();
        output = updateDrive();
        //not sure if Field relative is correct, but whatever
        drive.drive(output, 0, 0, false, true);

        SmartDashboard.putNumber("Angle of Elevation", elevationAngle);
        SmartDashboard.putNumber("Velocity", output);
        SmartDashboard.putNumber("Elevation velocity", elevationVelocity);
    }

    private double updateDrive() {
        //assuming we drive straight in the x direction for now
        return -(elevationAngle / elevationAngle)*(Math.pow(AutoConstants.polyCoeff * (Math.abs(elevationAngle)/AutoConstants.platformMaxAngle), 2)) * AutoConstants.balanceSpeed;
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
    public boolean isFinished(){
        return(Math.abs(elevationVelocity) > errorThreshold && (signOf(elevationAngle) != signOf(elevationVelocity)));
    }

}