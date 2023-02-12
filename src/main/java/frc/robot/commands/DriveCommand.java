package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//Do not use within first checkDuration * 1000 millis of the day
public class DriveCommand extends CommandBase{
    DriveSubsystem drive;
    double speed;
    boolean fieldRelative;
    double angleThreshold;

    double duration;
    double time;
    boolean check;

    double elevationAngle;
    Rotation2d heading;

    public DriveCommand(DriveSubsystem drive, double speed, boolean fieldRelative, double angleThreshold, double checkDuration){
        this.drive = drive;
        this.angleThreshold = angleThreshold;
        this.duration = checkDuration;
        this.speed = speed;
        this.fieldRelative = fieldRelative;
        check = false;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        elevationAngle = drive.getElevationAngle();
        heading = drive.getHeading();
    }

    @Override
    public void execute(){
        drive.drive(speed * heading.getCos(), speed * heading.getSin(), 0, fieldRelative, true);
        elevationAngle = drive.getElevationAngle();
        if(Math.abs(elevationAngle) > angleThreshold){
            check = true;
            time = System.currentTimeMillis();
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
