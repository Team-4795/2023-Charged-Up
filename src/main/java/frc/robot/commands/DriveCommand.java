package frc.robot.commands;

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
        this.duration = checkDuration;
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
        drive.drive(speed, 0, 0, true, true);
        elevationAngle = drive.getElevationAngle();
        if(Math.abs(elevationAngle) > angleThreshold){
            if(!check){
                time = System.currentTimeMillis();
                check = true;
            }
        } else {
            time = 0;
            check = false;
        }
    }

    @Override
    public boolean isFinished(){
        return (((System.currentTimeMillis() - time) > duration) && check);
    }
}
