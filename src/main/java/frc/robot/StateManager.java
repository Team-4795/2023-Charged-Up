package frc.robot;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LiftArm;
import frc.robot.Constants.DriveConstants;

public class StateManager {
    // What state were in
    State state;

    // Either what were picking or what were holding
    Object object;

    // If we are or are not storing an object
    // Temporary
    boolean storing = false;

    public StateManager() {
        this.state = State.StowLow;
        this.object = Object.None;
    }

    public void pickCube() {
        object = Object.Cube;
    }

    public void pickCone() {
        object = Object.Cone;
    }

    public void stow() {
        state = State.StowLow;
    }

    public void handleDpad(int angle) {
        if (storing) {
            switch (angle) {
                case 0: state = State.HighScoreCube; break;
                case 270: state = State.MidScore; break;
                case 180: state = State.LowScore; break;
                case 90: state = State.StowInFrame; break;
                default: break;
            }
        } else {
            switch (angle) {
                case 0: state = State.DoubleFeeder; break;
                case 270: state = State.SingleFeeder; break;
                case 180: state = State.LowPickup; break;
                case 90: state = State.StowInFrame; break;
                default: break;
            }
        }
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

    public Optional<Double> getArmSetpoint() {
        return this.state.get(this.object).map(setpoints -> setpoints.arm);
    }

    public Optional<Double> getIntakeSetpoint() {
        return this.state.get(this.object).map(setpoints -> setpoints.intake);
    }

    public State getState(){
        return state;
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
            case LowPickup: result = new Setpoints(0.94, 0.0); break;
            case SingleFeeder: break;
            case DoubleFeeder: break;
            case LowScore: result = new Setpoints(0.905, 0.0); break;
            case MidScore: result = new Setpoints(0.74, 0.0); break;
            case HighScoreCube: result = new Setpoints(.68, 0.0); break;
            case StowInFrame: result = new Setpoints(0.16, 0.0); break;
            case StowLow: result = new Setpoints(0.96, 0.0); break;
        }

        if (result != null) {
            result.intake = DriveConstants.kSlowCubeIntakeSpeed;
        }

        return Optional.ofNullable(result);
    }

    private Optional<Setpoints> getCone() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = new Setpoints(0.94, 0.0); break;
            case SingleFeeder: break;
            case DoubleFeeder: break;
            case LowScore: result = new Setpoints(0.89, 0.0); break;
            case MidScore: result = new Setpoints(0.7, 0.0); break;
            case HighScoreCube: break;
            case StowInFrame: result = new Setpoints(0.16, 0.0); break;
            case StowLow: result = new Setpoints(0.96, 0.0); break;
        }

        if (result != null) {
            result.intake = DriveConstants.kSlowConeIntakeSpeed;
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

    Setpoints(double arm, double wrist) {
        this.arm = arm;
        this.wrist = wrist;
        this.intake = 0.0;
    }
}