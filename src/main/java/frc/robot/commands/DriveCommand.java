package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//purely to test how fieldRelative affects the direction of Swerve
public class DriveCommand extends CommandBase{
    DriveSubsystem drive;
    double speed;
    double rotation;
    boolean fieldRelative;
    double elevationAngle;
    double angleThreshold;



    public DriveCommand(DriveSubsystem drive, double speed, double rotation, boolean fieldRelative, double angleThreshold){
        this.angleThreshold = angleThreshold;
        this.speed = speed;
        this.rotation = rotation;
        this.fieldRelative = fieldRelative;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        elevationAngle = drive.getElevationAngle();

    }

    @Override
    public void execute(){
        drive.drive(speed, 0, rotation, fieldRelative, true);
        elevationAngle = drive.getElevationAngle();
    }

    @Override
    public boolean isFinished(){
        //return (((System.currentTimeMillis() - time) > duration) && startDuration);
        return (Math.abs(elevationAngle) > angleThreshold);
    }
}