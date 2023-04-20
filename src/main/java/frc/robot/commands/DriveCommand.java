package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//Do not use within first checkDuration * 1000 millis of the day
public class DriveCommand extends CommandBase{
    DriveSubsystem drive;
    double speed;
    double angleThreshold;

    double duration;
    double time;
    boolean check;
    double elevationAngle;

    public DriveCommand(DriveSubsystem drive, double speed, double angleThreshold, double checkDuration){
        this.drive = drive;
        this.angleThreshold = angleThreshold;
        this.duration = 1000 * checkDuration;
        this.speed = speed;
        check = false;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        elevationAngle = drive.getElevationAngle();
    }

    @Override
    public void execute(){
        drive.drive(0, speed, 0, true, true);
        elevationAngle = drive.getElevationAngle();
        if(Math.abs(elevationAngle) > angleThreshold){
            if(!check){
                time = System.currentTimeMillis();
                check = true;
            }
        } else {
            check = false;
            time = 0;
        }
    }

    @Override
    public boolean isFinished(){
        return (((System.currentTimeMillis() - time) > duration) && check);
    }
}
