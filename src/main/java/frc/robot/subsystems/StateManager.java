package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateManager extends SubsystemBase {
    public LiftArm arm;

    // What state were in
    State state;

    // Either what were picking or what were holding
    Object object;

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
        if (storing()) {
            state = State.LowPickup;
        } else {
            state = State.LowScore;
        }

        setSetpoint();
    }

    // Single feeder or mid score
    public void button2() {
        if (storing()) {
            state = State.SingleFeeder;
        } else {
            state = State.MidScore;
        }

        setSetpoint();
    }

    // Double feeder or high score
    public void button3() {
        if (storing()) {
            state = State.DoubleFeeder;
        } else {
            state = State.HighScoreCube;
        }

        setSetpoint();
    }

    // temporary function, if we are storing an object
    public boolean storing() {
        return true;
    }

    private void setSetpoint() {
        state.get(object).ifPresent(setpoints -> { arm.setPosition(setpoints.arm); });
    }
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
            case LowPickup: result = new Setpoints(0.0, 0.0, 0.0);
            case SingleFeeder: result = new Setpoints(0.0, 0.0, 0.0);
            case DoubleFeeder: result = new Setpoints(0.0, 0.0, 0.0);
            case LowScore: result = new Setpoints(0.0, 0.0, 0.0);
            case MidScore: result = new Setpoints(0.0, 0.0, 0.0);
            case HighScoreCube: result = new Setpoints(0.0, 0.0, 0.0);
            case StowInFrame: result = new Setpoints(0.0, 0.0, 0.0);
            case StowLow: result = new Setpoints(0.0, 0.0, 0.0);            
        }

        return Optional.ofNullable(result);
    }

    private Optional<Setpoints> getCone() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = new Setpoints(0.0, 0.0, 0.0); break;
            case SingleFeeder: result = new Setpoints(0.0, 0.0, 0.0); break;
            case DoubleFeeder: result = new Setpoints(0.0, 0.0, 0.0); break;
            case LowScore: result = new Setpoints(0.0, 0.0, 0.0); break;
            case MidScore: result = new Setpoints(0.0, 0.0, 0.0); break;
            case HighScoreCube: break;
            case StowInFrame: result = new Setpoints(0.0, 0.0, 0.0); break;
            case StowLow: result = new Setpoints(0.0, 0.0, 0.0); break;     
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
    double joint;
    double intake;

    Setpoints(double arm, double joint, double intake) {
        this.arm = arm;
        this.joint = joint;
        this.intake = intake;
    }
}