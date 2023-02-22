package frc.robot;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.subsystems.LiftArm;
import frc.robot.Constants.CubeSetpointConstants;
import frc.robot.Constants.ConeSetpointConstants;

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
            case LowPickup: result = new Setpoints(CubeSetpointConstants.kLowPickupArm, CubeSetpointConstants.kLowPickupWrist,CubeSetpointConstants.kLowPickupIntake); break;
            case SingleFeeder: result = new Setpoints(CubeSetpointConstants.kSingleFeederArm, CubeSetpointConstants.kSingleFeederWrist, CubeSetpointConstants.kSingleFeederIntake); break;
            case DoubleFeeder: result = new Setpoints(CubeSetpointConstants.kDoubleFeederArm, CubeSetpointConstants.kDoubleFeederWrist, CubeSetpointConstants.kDoubleFeederIntake); break;
            case LowScore: result = new Setpoints(CubeSetpointConstants.kLowScoreArm, CubeSetpointConstants.kLowScoreWrist, CubeSetpointConstants.kLowScoreIntake); break;
            case MidScore: result = new Setpoints(CubeSetpointConstants.kMidScoreArm, CubeSetpointConstants.kMidScoreWrist, CubeSetpointConstants.kMidScoreIntake); break;
            case HighScoreCube: result = new Setpoints(CubeSetpointConstants.kHighScoreArm, CubeSetpointConstants.kHighScoreWrist, CubeSetpointConstants.kHighScoreIntake); break;
            case StowInFrame: result = new Setpoints(CubeSetpointConstants.kStowInFrameArm, CubeSetpointConstants.kStowInFrameWrist, CubeSetpointConstants.kStowInFrameIntake); break;
            case StowLow: result = new Setpoints(CubeSetpointConstants.kStowLowArm, CubeSetpointConstants.kStowLowWrist, CubeSetpointConstants.kStowLowIntake); break;
        }

        return Optional.ofNullable(result);
    }

    private Optional<Setpoints> getConeSetpoints() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = new Setpoints(ConeSetpointConstants.kLowPickupArm, ConeSetpointConstants.kLowPickupWrist,ConeSetpointConstants.kLowPickupIntake); break;
            case SingleFeeder: result = new Setpoints(ConeSetpointConstants.kSingleFeederArm, ConeSetpointConstants.kSingleFeederWrist, ConeSetpointConstants.kSingleFeederIntake); break;
            case DoubleFeeder: result = new Setpoints(ConeSetpointConstants.kDoubleFeederArm, ConeSetpointConstants.kDoubleFeederWrist, ConeSetpointConstants.kDoubleFeederIntake); break;
            case LowScore: result = new Setpoints(ConeSetpointConstants.kLowScoreArm, ConeSetpointConstants.kLowScoreWrist, ConeSetpointConstants.kLowScoreIntake); break;
            case MidScore: result = new Setpoints(ConeSetpointConstants.kMidScoreArm, ConeSetpointConstants.kMidScoreWrist, ConeSetpointConstants.kMidScoreIntake); break;
            case HighScoreCube: result = new Setpoints(ConeSetpointConstants.kHighScoreArm, ConeSetpointConstants.kHighScoreWrist, ConeSetpointConstants.kHighScoreIntake); break;
            case StowInFrame: result = new Setpoints(ConeSetpointConstants.kStowInFrameArm, ConeSetpointConstants.kStowInFrameWrist, ConeSetpointConstants.kStowInFrameIntake); break;
            case StowLow: result = new Setpoints(ConeSetpointConstants.kStowLowArm, ConeSetpointConstants.kStowLowWrist, ConeSetpointConstants.kStowLowIntake); break;
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