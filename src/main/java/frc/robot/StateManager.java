package frc.robot;
import java.util.Optional;
import java.util.function.BooleanSupplier;

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
    private BooleanSupplier isStoring;
    private boolean overrideStoring;


    public enum LED {
        Cone,
        Cube,
    }

    public enum Gamepiece {
        Cube,
        Cone,
        None,
    }

    public StateManager(BooleanSupplier isStoring) {
        this.state = State.StowLow;
        this.gamepiece = Gamepiece.None;
        this.isStoring = isStoring;
    }

    public void pickCube() {
        gamepiece = Gamepiece.Cube;

        SmartDashboard.putString("Gamepiece", "Cube");
    }

    public void pickCone() {
        gamepiece = Gamepiece.Cone;

        SmartDashboard.putString("Gamepiece", "Cone");
    }

    public void stow() {
        state = State.StowLow;
    }

    public void handleDpad(int angle) {
        if (isStoring.getAsBoolean() ^ overrideStoring) {
            switch (angle) {
                case 0: state = State.HighScore; break;
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

    public Optional<Double> getArmSetpoint() {
        return this.state.getSetpoints(this.gamepiece).map(setpoints -> setpoints.arm);
    }

    public Optional<Double> getOuttakeSetpoint() {
        return this.state.getSetpoints(this.gamepiece).map(setpoints -> setpoints.outtake);
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

    public Gamepiece getGamepiece() {
        return this.gamepiece;
    }
}

enum State {
    LowPickup,
    SingleFeeder,
    DoubleFeeder,
    LowScore,
    MidScore,
    HighScore,
    StowInFrame,
    StowLow;

    private Optional<Setpoints> getCubeSetpoints() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = new Setpoints(0.93, false, -0.3); break;
            case SingleFeeder: result = new Setpoints(0.7, false, -0.3); break;
            case DoubleFeeder: result = new Setpoints(0.64, true, -0.3); break;
            case LowScore: result = new Setpoints(0.89, false, -0.3); break;
            case MidScore: result = new Setpoints(0.69, true, -0.3); break;
            case HighScore: result = new Setpoints(.62, true, -0.3); break;
            case StowInFrame: result = new Setpoints(0.16, false, -0.3); break;
            case StowLow: result = new Setpoints(0.96, false, -0.3); break;
        }

        return Optional.ofNullable(result);
    }

    private Optional<Setpoints> getConeSetpoints() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = new Setpoints(0.94, false, -0.4); break;
            case SingleFeeder: result = new Setpoints(0.6, false, -0.4); break;
            case DoubleFeeder: result = new Setpoints(0.64, true, -0.4); break;
            case LowScore: result = new Setpoints(0.87, false, -0.4); break;
            case MidScore: result = new Setpoints(0.71, false, -0.4); break;
            case HighScore: result = new Setpoints(.64, true, -0.4); break;
            case StowInFrame: result = new Setpoints(0.16, false, -0.4); break;
            case StowLow: result = new Setpoints(0.96, false, -0.4); break;
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
    double outtake;

    Setpoints(double arm, boolean wrist, double outtake) {
        this.arm = arm;
        this.wrist = wrist;
        this.outtake = outtake;
    }
}