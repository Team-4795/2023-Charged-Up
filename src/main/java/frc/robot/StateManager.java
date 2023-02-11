package frc.robot;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LiftArm;

public class StateManager {
    public LiftArm arm;

    // What state were in
    State state;

    // Either what were picking or what were holding
    Object object;

    // If we are or are not storing an object
    // Temporary
    boolean storing = false;

    Setpoints current_setpoint = new Setpoints(0.0, 0.0, 0.0);

    public StateManager(LiftArm arm) {
        this.state = State.StowLow;
        this.object = Object.None;
        this.arm = arm;
    }

    public void pickCube() {
        object = Object.Cube;
    }

    public void pickCone() {
        object = Object.Cone;
    }

    public void stow() {
        state = State.StowLow;
        setSetpoint();
    }

    // Low pickup or low score
    public void button1() {
        if (storing) {
            state = State.LowScore;
        } else {
            state = State.LowPickup;
        }

        setSetpoint();
    }

    // Single feeder or mid score
    public void button2() {
        if (storing) {
            state = State.MidScore;
        } else {
            state = State.SingleFeeder;
        }

        setSetpoint();
    }

    // Double feeder or high score
    public void button3() {
        if (storing) {
            state = State.HighScoreCube;
        } else {
            state = State.DoubleFeeder;
        }

        setSetpoint();
    }

    public void toggleStoring() {
        storing = !storing;
    }

    public void setStoring() {
        storing = true;
    }

    public void setNotStoring() {
        storing = false;
    }

    private void setSetpoint() {
        state.get(object).ifPresent(setpoints -> { 
            current_setpoint = setpoints;
            arm.setPosition(setpoints.arm);
         });
    }

    // @Override
    // public void periodic() {
    //     SmartDashboard.putNumber("Arm setpoint", setpoint_debug.arm);
    // }
}

enum State {
    LowPickup,
    SingleFeeder,
    DoubleFeeder,
    LowScore,
    MidScore,
    HighScoreCube,
    StowInFrame,
    StowLow;

    private Optional<Setpoints> getCube() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = new Setpoints(0.94, 0.0, 0.0); break;
            case SingleFeeder: break;
            case DoubleFeeder: break;
            case LowScore: result = new Setpoints(0.907, 0.0, 0.0); break;
            case MidScore: result = new Setpoints(0.772, 0.0, 0.0); break;
            case HighScoreCube: result = new Setpoints(0.75, 0.0, 0.0); break;
            case StowInFrame: result = new Setpoints(0.16, 0.0, 0.0); break;
            case StowLow: result = new Setpoints(0.96, 0.0, 0.0); break;
        }

        return Optional.ofNullable(result);
    }

    private Optional<Setpoints> getCone() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = new Setpoints(0.94, 0.0, 0.0); break;
            case SingleFeeder: break;
            case DoubleFeeder: break;
            case LowScore: result = new Setpoints(0.907, 0.0, 0.0); break;
            case MidScore: result = new Setpoints(0.714, 0.0, 0.0); break;
            case HighScoreCube: break;
            case StowInFrame: result = new Setpoints(0.16, 0.0, 0.0); break;
            case StowLow: result = new Setpoints(0.96, 0.0, 0.0); break;
        }

        return Optional.ofNullable(result);
    }

    public Optional<Setpoints> get(Object object) {
        switch (object) {
            case Cube: return getCube();
            case Cone: return getCone();
            default: return Optional.empty();
        }
    }
}

enum Object {
    Cube,
    Cone,
    None,
}

class Setpoints {
    double arm;
    double wrist;
    double intake;

    Setpoints(double arm, double wrist, double intake) {
        this.arm = arm;
        this.wrist = wrist;
        this.intake = intake;
    }
}