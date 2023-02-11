package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;


public class AutoBalance extends CommandBase{
    DriveSubsystem drive;
    double elevationAngle;
    double heading;
    double errorThreshold;
    double checkDuration;
    //ArrayList<Double> checkZero;
    double[] output;

    /*DataLog balanceLog;
    DoubleLogEntry elevationLog;
    DoubleLogEntry headingLog;
    DoubleLogEntry xVelocity;
    DoubleLogEntry yVelocity;*/

    public AutoBalance(DriveSubsystem drive, double errorThreshold, double checkDuration){
        //Check duration in millis
        this.errorThreshold = errorThreshold;
        this.drive = drive;
        this.checkDuration = checkDuration;
        output = new double[3];
        //checkZero = new ArrayList<>();
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        /*Backup plan for when the thing starts blowing up for absolutely no reason and SmartDashboard aint good enough
        haven't tested not calling DataLogManager.start() in Robot.init but should work regardless
        hopefully won't need this during testing, but... */

        /*DataLogManager.start();
        balanceLog = DataLogManager.getLog();
        elevationLog = new DoubleLogEntry(balanceLog, "/elevationAngle");
        headingLog = new DoubleLogEntry(balanceLog, "/headingAngle");
        xVelocity = new DoubleLogEntry(balanceLog, "/xVelocity");
        yVelocity = new DoubleLogEntry(balanceLog, "/yVelocity");*/

        /*for(int i = 0; i < 1 + (int) Math.round(checkDuration / 20); i++){
            checkZero.add(1000.0); //just not under the error threshold
        }*/
    }

    @Override
    public void execute(){
        elevationAngle = drive.getElevationAngle();
        heading = drive.getHeading().getDegrees(); // in degrees
        output = updateDrive();

        //remove oldest element, add newest
        //checkZero.remove(0);
        //checkZero.add(elevationAngle);

        //not field relative so that the wheels turn forward in relation to the heading of the robot
        drive.drive(output[0], output[1], output[2], false, true);
        SmartDashboard.putNumber("Angle of Elevation", elevationAngle);
        SmartDashboard.putNumber("X velocity", output[0]);
        SmartDashboard.putNumber("Y velocity", output[1]);
        SmartDashboard.putNumber("Heading", heading);

        /*elevationLog.append(elevationAngle);
        headingLog.append(heading);
        xVelocity.append(output[0]);
        yVelocity.append(output[1]);*/
    }

    private double[] updateDrive() {
        //accounts for orientation of robot in relation to charge station; negative signs could be flipped
        double[] driveValues = new double[3];
        double speed;
        double angle;
        //may need to add a speed = 0 when angle = 0
        speed = (Math.abs(elevationAngle) / AutoConstants.platformMaxAngle) * (Math.abs(elevationAngle) / AutoConstants.platformMaxAngle) * AutoConstants.balanceSpeed;
        if(elevationAngle < 0){
            speed *= -1;
        }
        angle = Math.toRadians(heading);
        driveValues[0] = speed * Math.cos(angle);
        driveValues[1] = speed * Math.sin(angle);
        return driveValues;
    }

    @Override
    public boolean isFinished(){
        /*for(double angle : checkZero){
            if(Math.abs(angle) > errorThreshold){
                return false;
            }
        }*/

        //balanceLog.close(); 

        return (Math.abs(elevationAngle) < errorThreshold);
    }

}
