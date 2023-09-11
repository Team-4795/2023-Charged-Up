package frc.robot.subsystems.intake;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateManager.State;
import frc.robot.StateManager;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private static Intake mInstance;

    public static Intake getInstance() {
        if (mInstance == null) {
            switch (Constants.getRobot()) {
                case Comp:
                    mInstance = new Intake(new IntakeIOSparkMax());
                    break;
                case Sim:
                    mInstance = new Intake(new IntakeIO() {});
                    break;
            }
        }

        return mInstance;
    }

    // Speed to spin at
    public double setpoint = 0.0;

    // Current sensing storing boolean
    private static boolean currentStoring = false;

    // Outtake speed
    private double outtakeSpeed = 0.0;

    // If true, flip the state of `storing`
    private static boolean overrideStoring = false;

    // Circular buffer used for averaging current
    private CircularBuffer currentValues = new CircularBuffer(IntakeConstants.currentAvgSize);

    public Intake(IntakeIO io) {
        this.io = io;

        setDefaultCommand(intakeCommand());
    }

    // Run intake
    public void intake() {
        double speed = 0;

        if (isStoring()) {
            switch (StateManager.getInstance().getGamepiece()) {
                case Cube:
                    speed = IntakeConstants.kCubeSlowIntakeSpeed;
                    break;
                case Cone:
                    speed = IntakeConstants.kConeSlowIntakeSpeed;
                    break;
                default:
                    break;
            }
        } else {
            switch (StateManager.getInstance().getGamepiece()) {
                case Cube:
                    speed = IntakeConstants.kCubeIntakeSpeed;
                    break;
                case Cone:
                    speed = IntakeConstants.kConeIntakeSpeed;
                    break;
                default:
                    break;
            }
            if(StateManager.getInstance().getState() == State.StowInFrame){
                speed = 0;
            }
        }

        io.setOutput(speed);
    }

    public void setOuttakeSpeed(double speed) {
        this.outtakeSpeed = speed;
    }

    // Run outtake
    public void outtake() {
        io.setOutput(outtakeSpeed);
    }

    public void resetStoring() {
        currentStoring = false;
    }

    public static boolean isStoring() {
        if (DriverStation.isAutonomous()) {
            return overrideStoring;
        } else {
            return currentStoring ^ overrideStoring;
        }
    }

    public void setOverrideStoring(boolean override) {
        overrideStoring = override;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);

        if (inputs.currentAmps > 2) {
            currentValues.addLast(inputs.currentAmps);
        }

        if (avgCurrent() > IntakeConstants.storingCurrentThreshold) {
            currentStoring = true;
        } else {
            currentStoring = false;
        }

    }

    public Command intakeCommand() {
        return run(this::intake);
    }

    public Command outtakeCommand() {
        return outtakeCommand(false);
    }

    public Command outtakeCommand(boolean forever) {
        // If not `forever`, stop outtake after 0.25s
        return run(this::outtake)
                .raceWith(forever ? Commands.run(() -> {}) : new WaitCommand(0.25))
                .finallyDo((interrupted) -> intake());
    }

    private double avgCurrent() {
        double sum = 0;
        for (int i = 0; i < currentValues.size(); i++) {
            sum += currentValues.get(i);
        }
        return sum / currentValues.size();
    }
}
