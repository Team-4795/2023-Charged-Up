package frc.robot;
import java.util.Optional;

import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;

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
        this.getArmSetpoint().ifPresent(arm::setPosition);
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
            case LowPickup: result = new Setpoints(CubeSetpointConstants.kLowPickupArm, CubeSetpointConstants.kLowPickupWrist,CubeSetpointConstants.kLowPickupOuttake); break;
            case SingleFeeder: result = new Setpoints(CubeSetpointConstants.kStowHighArm, CubeSetpointConstants.kStowHighWrist, CubeSetpointConstants.kStowHighOuttake); break;
            case DoubleFeeder: result = new Setpoints(CubeSetpointConstants.kDoubleFeederArm, CubeSetpointConstants.kDoubleFeederWrist, CubeSetpointConstants.kDoubleFeederOuttake); break;
            case LowScore: result = new Setpoints(CubeSetpointConstants.kLowScoreArm, CubeSetpointConstants.kLowScoreWrist, CubeSetpointConstants.kLowScoreOuttake); break;
            case MidScore: result = new Setpoints(CubeSetpointConstants.kMidScoreArm, CubeSetpointConstants.kMidScoreWrist, CubeSetpointConstants.kMidScoreOuttake); break;
            case HighScore: result = new Setpoints(CubeSetpointConstants.kHighScoreArm, CubeSetpointConstants.kHighScoreWrist, CubeSetpointConstants.kHighScoreOuttake); break;
            case StowInFrame: result = new Setpoints(CubeSetpointConstants.kStowInFrameArm, CubeSetpointConstants.kStowInFrameWrist, CubeSetpointConstants.kStowInFrameOuttake); break;
            case StowLow: result = new Setpoints(CubeSetpointConstants.kStowLowArm, CubeSetpointConstants.kStowLowWrist, CubeSetpointConstants.kStowLowOuttake); break;
        }

        return Optional.ofNullable(result);
    }

    private Optional<Setpoints> getConeSetpoints() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = new Setpoints(ConeSetpointConstants.kLowPickupArm, ConeSetpointConstants.kLowPickupWrist,ConeSetpointConstants.kLowPickupOuttake); break;
            case SingleFeeder: result = new Setpoints(ConeSetpointConstants.kStowHighArm, ConeSetpointConstants.kStowHighWrist, ConeSetpointConstants.kStowHighOuttake); break;
            case DoubleFeeder: result = new Setpoints(ConeSetpointConstants.kDoubleFeederArm, ConeSetpointConstants.kDoubleFeederWrist, ConeSetpointConstants.kDoubleFeederOuttake); break;
            case LowScore: result = new Setpoints(ConeSetpointConstants.kLowScoreArm, ConeSetpointConstants.kLowScoreWrist, ConeSetpointConstants.kLowScoreOuttake); break;
            case MidScore: result = new Setpoints(ConeSetpointConstants.kMidScoreArm, ConeSetpointConstants.kMidScoreWrist, ConeSetpointConstants.kMidScoreOuttake); break;
            case HighScore: result = new Setpoints(ConeSetpointConstants.kHighScoreArm, ConeSetpointConstants.kHighScoreWrist, ConeSetpointConstants.kHighScoreOuttake); break;
            case StowInFrame: result = new Setpoints(ConeSetpointConstants.kStowInFrameArm, ConeSetpointConstants.kStowInFrameWrist, ConeSetpointConstants.kStowInFrameOuttake); break;
            case StowLow: result = new Setpoints(ConeSetpointConstants.kStowLowArm, ConeSetpointConstants.kStowLowWrist, ConeSetpointConstants.kStowLowOuttake); break;
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
