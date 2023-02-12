package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//purely to test how fieldRelative affects the direction of Swerve
public class DriveCommandOld extends CommandBase{
    DriveSubsystem drive;
    double speed;
    boolean fieldRelative;
    double angleThreshold;

    double elevationAngle;

    public DriveCommandOld(DriveSubsystem drive, double speed, boolean fieldRelative, double angleThreshold){
        this.drive = drive;
        this.angleThreshold = angleThreshold;
        this.speed = speed;
        this.fieldRelative = fieldRelative;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        elevationAngle = drive.getElevationAngle();
    }

    @Override
    public void execute(){
        drive.drive(speed, 0, 0, fieldRelative, true);
        elevationAngle = drive.getElevationAngle();
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(elevationAngle) > angleThreshold);
    }
}