package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

// Only drives straight in the X-direction, assumes platform in front of robot
public class DriveCommand extends CommandBase {
    Drive drive = Drive.getInstance();
    double speed;
    double angleThreshold;

    double duration;
    double time;
    boolean check;

    double elevationAngle;

    public DriveCommand(double speed, double angleThreshold, double checkDuration) {
        this.angleThreshold = angleThreshold;
        this.speed = speed;
        this.duration = checkDuration;
        this.time = 0;
        this.check = false;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        elevationAngle = drive.getElevationAngle();
    }

    @Override
    public void execute() {
        drive.runVelocity(new ChassisSpeeds(speed, 0, 0));
        elevationAngle = drive.getElevationAngle();
        if (Math.abs(elevationAngle) > angleThreshold) {
            if (!check) {
                time = Timer.getFPGATimestamp() + 100;
                check = true;
            }
        } else {
            check = false;
            time = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return (((Timer.getFPGATimestamp() + 100 - time) > duration) && check);
    }
}
