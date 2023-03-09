package frc.robot;
import java.util.Optional;

import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;
import frc.utils.Setpoints;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CubeSetpointConstants;
import frc.robot.Constants.ConeSetpointConstants;

public class StateManager {
    private Vision vision;
    private LiftArm arm;
    private EndEffectorIntake intake;
    private LEDs leds;

    // What state were in
    private State state;

    // Either what were picking or what were storing
    private Gamepiece gamepiece;

    public enum LED {
        Cone,
        Cube,
    }

    public enum Gamepiece {
        Cube,
        Cone,
        None,
    }

    public StateManager(Vision vision, LiftArm arm, EndEffectorIntake intake, LEDs leds) {
        this.state = State.StowInFrame;
        this.gamepiece = Gamepiece.None;

        this.vision = vision;
        this.arm = arm;
        this.intake = intake;
        this.leds = leds;
    }

    public void pickCube() {
        gamepiece = Gamepiece.Cube;
        SmartDashboard.putString("Gamepiece", "Cube");
        leds.setRGB(127, 0, 255);
        vision.switchToTag();
    }

    public void pickCone() {
        gamepiece = Gamepiece.Cone;
        SmartDashboard.putString("Gamepiece", "Cone");
        leds.setRGB(255, 255, 0);

        vision.switchToTape();
    }

    public void dpadUp() {
        if (intake.isStoring()) {
            state = State.HighScore;
        } else {
            state = State.DoubleFeeder;
        }

        setSetpoints();
    }

    public void dpadLeft() {
        if (intake.isStoring()) {
            state = State.MidScore;
        } else {
            state = State.SingleFeeder;
        }

        setSetpoints();
    }
    
    public void dpadDown() {
        if (intake.isStoring()) {
            state = State.LowScore;
        } else {
            state = State.LowPickup;
        }

        setSetpoints();
    }

    public void dpadRight() {
        state = State.StowInFrame;

        setSetpoints();
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

    public boolean isStowing() {
        switch (this.state) {
            case StowInFrame: return true;
            default: return false;
        }
    }

    public Gamepiece getGamepiece() {
        return this.gamepiece;
    }

    private void setSetpoints() {
        this.getArmSetpoint().ifPresent(arm::setTargetPosition);
        this.getOuttakeSetpoint().ifPresent(intake::setOuttakeSpeed);
        this.getWristExtended().ifPresent(intake::setExtendedTarget);

        SmartDashboard.putString("State", state.name());
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
            case LowPickup: result = CubeSetpointConstants.kLowPickup; break;
            case SingleFeeder: result = CubeSetpointConstants.kStowHigh; break;
            case DoubleFeeder: result = CubeSetpointConstants.kDoubleFeeder; break;
            case LowScore: result = CubeSetpointConstants.kLowScore; break;
            case MidScore: result = CubeSetpointConstants.kMidScore; break;
            case HighScore: result = CubeSetpointConstants.kHighScore; break;
            case StowInFrame: result = CubeSetpointConstants.kStowInFrame; break;
            case StowLow: result = CubeSetpointConstants.kStowLow; break;
        }

        return Optional.ofNullable(result);
    }

    private Optional<Setpoints> getConeSetpoints() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = ConeSetpointConstants.kLowPickup; break;
            case SingleFeeder: result = ConeSetpointConstants.kStowHigh; break;
            case DoubleFeeder: result = ConeSetpointConstants.kDoubleFeeder; break;
            case LowScore: result = ConeSetpointConstants.kLowScore; break;
            case MidScore: result = ConeSetpointConstants.kMidScore; break;
            case HighScore: result = ConeSetpointConstants.kHighScore; break;
            case StowInFrame: result = ConeSetpointConstants.kStowInFrame; break;
            case StowLow: result = ConeSetpointConstants.kStowLow; break;
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