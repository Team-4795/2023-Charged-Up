package frc.robot;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.subsystems.LiftArm;
import frc.robot.Constants.DriveConstants;

public class StateManager {
    // What state were in
    private State state;

    // Either what were picking or what were holding
    private Gamepiece gamepiece;

    // If we are or are not storing a gamepiece
    // Temporary
    private boolean storing = false;

    public enum LED {
        Cone,
        Cube,
    }

    enum Gamepiece {
        Cube,
        Cone,
        None,
    }

    public StateManager() {
        this.state = State.StowLow;
        this.gamepiece = Gamepiece.None;
    }

    public void pickCube() {
        gamepiece = Gamepiece.Cube;
    }

    public void pickCone() {
        gamepiece = Gamepiece.Cone;
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

        SmartDashboard.putString("State", state.name());
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
        return this.state.getSetpoints(this.gamepiece).map(setpoints -> setpoints.arm);
    }

    public Optional<Double> getIntakeSetpoint() {
        return this.state.getSetpoints(this.gamepiece).map(setpoints -> setpoints.intake);
    }

    public Optional<LED> getLED() {
        return this.state.getLED(this.gamepiece);
    }

    public Optional<Boolean> getWristExtended() {
        return this.state.getSetpoints(this.gamepiece).map(setpoints -> setpoints.wrist);
    }

    public State getState() {
        return this.state;
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

    private Optional<Setpoints> getCubeSetpoints() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = new Setpoints(0.933, false, 0.5); break;
            case SingleFeeder: result = new Setpoints(0.7, false, 0.5); break;
            case DoubleFeeder: result = new Setpoints(0.64, true, 0.5); break;
            case LowScore: result = new Setpoints(0.91, false, 0.1); break;
            case MidScore: result = new Setpoints(0.7, true, 0.1); break;
            case HighScoreCube: result = new Setpoints(.64, true, 0.1); break;
            case StowInFrame: result = new Setpoints(0.16, false, 0.1); break;
            case StowLow: result = new Setpoints(0.96, false, 0.1); break;
        }

        return Optional.ofNullable(result);
    }

    private Optional<Setpoints> getConeSetpoints() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = new Setpoints(0.955, false, 1.0); break;
            case SingleFeeder: result = new Setpoints(0.6, false, 1.0); break;
            case DoubleFeeder: result = new Setpoints(0.64, true, 1.0); break;
            case LowScore: result = new Setpoints(0.87, false, 0.1); break;//not really tested
            case MidScore: result = new Setpoints(0.73, false, 0.1); break;
            case HighScoreCube: result = new Setpoints(.64, true, 0.1); break;
            case StowInFrame: result = new Setpoints(0.16, false, 0.1); break;
            case StowLow: result = new Setpoints(0.96, false, 0.1); break;
        }

        return Optional.ofNullable(result);
    }

    public Optional<Setpoints> getSetpoints(StateManager.Gamepiece gamepiece) {
        switch (gamepiece) {
            case Cube: return getCubeSetpoints();
            case Cone: return getConeSetpoints();
            default: return Optional.empty();
        }
    }

    public Optional<StateManager.LED> getLED(StateManager.Gamepiece gamepiece) {
        switch (gamepiece) {
            case Cube: return Optional.of(StateManager.LED.Cube);
            case Cone: return Optional.of(StateManager.LED.Cone);
            default: return Optional.empty();
        }
    }
}

class Setpoints {
    double arm;
    boolean wrist;
    double intake;

    Setpoints(double arm, boolean wrist, double intake) {
        this.arm = arm;
        this.wrist = wrist;
        this.intake = intake;
    }
}