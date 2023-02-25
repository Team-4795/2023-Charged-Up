package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//Only drives straight in the X-direction, assumes platform in front of robot
public class DriveCommandOld extends CommandBase{
    DriveSubsystem drive;
    double speed;
    double angleThreshold;

    double elevationAngle;

    public DriveCommandOld(DriveSubsystem drive, double speed, double angleThreshold){
        this.drive = drive;
        this.angleThreshold = angleThreshold;
        this.speed = speed;
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
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(elevationAngle) > angleThreshold);
    }
}