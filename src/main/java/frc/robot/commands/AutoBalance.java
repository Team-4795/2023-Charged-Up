package frc.robot.Commands;

import com.fasterxml.jackson.core.io.OutputDecorator;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.drive.Drive;

public class AutoBalance extends CommandBase {
    Swerve drive = Swerve.getInstance();
    double elevationAngle;
    double errorThreshold;
    double output;

    int previousSign = 0;
    int currentSign = 0;

    int signCheck = 0;
    boolean checkingOscillation = false;
    int oscillations = 1;

    Timer oscillationTimer = new Timer();

    public AutoBalance(double errorThreshold) {
        this.errorThreshold = errorThreshold;
        output = 0;
        oscillationTimer.reset();
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        elevationAngle = drive.getElevationAngle();
    }

    @Override
    public void execute() {
        elevationAngle = deadband(drive.getElevationAngle());
        if (elevationAngle < -AutoConstants.platformMaxAngle) {
            elevationAngle = -AutoConstants.platformMaxAngle;
        } else if (elevationAngle > AutoConstants.platformMaxAngle) {
            elevationAngle = AutoConstants.platformMaxAngle;
        }
        output = updateDrive();
        countOscillations();
        // not sure if Field relative is correct, but whatever
        drive.runVelocity(new ChassisSpeeds(0, output, 0));
        // drive.drive(output, 0, 0, false, true);
        drive.setBalanceSpeed(output);
        // drive.setOscillations(oscillations);
    }

    private double updateDrive() {
        // assuming we drive straight in the x direction for now
        return -signOf(elevationAngle)
                * (Math.pow(
                        AutoConstants.polyCoeff
                                / oscillations
                                * (Math.abs(elevationAngle) / AutoConstants.platformMaxAngle),
                        2))
                * AutoConstants.balanceSpeed;
    }

    private int signOf(double num) {
        if (num < 0) {
            return -1;
        } else if (num > 0) {
            return 1;
        } else {
            return 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // drive.setBalanceSpeed(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double deadband(double value) {
        if (-AutoConstants.deadbandValue < value && value < AutoConstants.deadbandValue) {
            return 0.0;
        }
        return value;
    }

    private void countOscillations() {
        previousSign = currentSign;
        currentSign = signOf(elevationAngle);
        if (previousSign != currentSign && !checkingOscillation) {
            signCheck = currentSign;
            oscillationTimer.start();
            checkingOscillation = true;
        }
        if (oscillationTimer.hasElapsed(AutoConstants.oscillationTime)) {
            if (currentSign == signCheck) {
                oscillations++;
            }
            oscillationTimer.stop();
            oscillationTimer.reset();
            checkingOscillation = false;
        }
    }
}
