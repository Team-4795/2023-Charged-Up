package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;


public class AutoBalance extends PIDCommand{
    DriveSubsystem drive;
    double velocityErrorThreshold;

    public AutoBalance(DriveSubsystem drive, double targetAngle, double velocityErrorThreshold) {
        super(new PIDController(0.3, 0.3, 0.3), 
        // get Gyro Angle
        drive::getElevationAngle, 
        //target Angle
        targetAngle, 
        //output to swerve drive command
        output -> drive.setModuleStates(convertSwerve(output)), 
        drive);
        this.drive = drive;
        this.velocityErrorThreshold = velocityErrorThreshold;
    }


    private static SwerveModuleState[] convertSwerve(double output) {
        //assuming that 0 angle is forward
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for(int i = 0; i < swerveModuleStates.length; i++){
            swerveModuleStates[i] = new SwerveModuleState(output, Rotation2d.fromDegrees(0.0));
        }
        return swerveModuleStates;
    }


    @Override
    public boolean isFinished(){
        //change so that it's finished when angle = 0 and angular velocity = 0
        return (Math.abs(drive.getElevationVelocity()) < velocityErrorThreshold && getController().atSetpoint());
    }

}
