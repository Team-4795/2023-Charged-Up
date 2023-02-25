package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//Only drives straight in the X-direction, assumes platform in front of robot
public class DriveCommandOld extends CommandBase{
    DriveSubsystem drive;
    double speed;
    double angleThreshold;

    double duration;
    double time;
    boolean check;
    double elevationAngle;

    public DriveCommandOld(DriveSubsystem drive, double speed, double angleThreshold, double checkDuration){
        this.drive = drive;
        this.angleThreshold = angleThreshold;
        this.speed = speed;
        this.duration = 1000 * checkDuration;
        this.time = 0;
        this.check = false;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        elevationAngle = drive.getElevationAngle();
    }

    @Override
    public void execute(){
        drive.drive(speed, 0, 0, false, true);
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