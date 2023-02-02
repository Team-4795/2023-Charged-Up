package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//purely to test how fieldRelative affects the direction of Swerve
public class DriveCommand extends CommandBase{
    DriveSubsystem drive;
    double time;
    double duration;
    double x;
    double y;
    double rotation;
    boolean fieldRelative;


    public DriveCommand(DriveSubsystem drive, double x, double y, double rotation, boolean fieldRelative, double duration){
        this.duration = 1000 * duration;
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        this.fieldRelative = fieldRelative;
        time = System.currentTimeMillis();
        addRequirements(drive);
    }

    @Override
    public void execute(){
        drive.drive(x, y, rotation, fieldRelative);
    }

    @Override
    public boolean isFinished(){
        return ((System.currentTimeMillis() - time) > duration);
    }
}