package frc.robot;
import java.util.Optional;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.utils.Setpoints;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CubeSetpointConstants;
import frc.robot.Constants.ConeSetpointConstants;

public class StateManager {
    private Vision vision;
    private LiftArm arm;
    private EndEffectorIntake intake;
    private LEDs leds;
    private DriveSubsystem drive;
    private Wrist wrist;

    // What state were in
    private State state;

    // Either what were picking or what were storing
    private static Gamepiece gamepiece = Gamepiece.None;

    public enum LED {
        Cone,
        Cube,
    }

    public enum Gamepiece {
        Cube,
        Cone,
        None,
    }

    public StateManager(Vision vision, LiftArm arm, EndEffectorIntake intake, LEDs leds, DriveSubsystem drive, Wrist wrist) {
        this.state = State.StowInFrame;

        this.vision = vision;
        this.arm = arm;
        this.intake = intake;
        this.leds = leds;
        this.drive = drive;
        this.wrist = wrist;
    }

    public void pickCube() {
        gamepiece = Gamepiece.Cube;
        SmartDashboard.putString("Gamepiece", "Cube");
        leds.setBottomRGB(127, 0, 255);
        vision.switchToTag();
    }

    public void pickCone() {
        gamepiece = Gamepiece.Cone;
        SmartDashboard.putString("Gamepiece", "Cone");
        leds.setBottomRGB(255, 255, 0);

        vision.switchToTape();
    }
    

    public void dpadUp() {
        if (intake.isStoring()) {
            if (Math.cos(Math.toRadians(drive.getAngle())) < 0) {
                state = State.BackwardsHighScore;
            } else {
                state = State.HighScore;
            }
        } else {
            if (Math.cos(Math.toRadians(drive.getAngle())) < 0) {
                state = State.DoubleFeeder;
            } else {
                state = State.BackwardsDoubleFeeder;
            }
        }

        setSetpoints();
    }

    public void dpadLeft() {
        if (intake.isStoring()) {
            if (Math.cos(Math.toRadians(drive.getAngle())) < 0) {
                state = State.BackwardsMidScore;
            } else {
                state = State.MidScore;
            }
        } else {
            // Switched to B for stow high
            // state = State.StowHigh;
        }

        setSetpoints();
    }
    
    public void dpadDown() {
        if (intake.isStoring()) {
            if (Math.cos(Math.toRadians(drive.getAngle())) < 0) {
                state = State.BackwardsLowScore;
            } else {
                state = State.LowScore;
            }
        } else {
            state = State.LowPickup;
        }

        setSetpoints();
    }

    public void dpadRight() {
        state = State.StowInFrame;

        setSetpoints();
    }

    public void stowHigh() {
        state = State.StowHigh;

        setSetpoints();
    }

    public Optional<Double> getArmSetpoint() {
        return this.state.getSetpoints().map(setpoints -> setpoints.arm);
    }

    public Optional<Double> getOuttakeSetpoint() {
        return this.state.getSetpoints().map(setpoints -> setpoints.outtake);
    }

    public Optional<Boolean> getWristExtended() {
        return this.state.getSetpoints().map(setpoints -> setpoints.wrist);
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

    public static Gamepiece getGamepiece() {
        return gamepiece;
    }

    private void setSetpoints() {
        this.getArmSetpoint().ifPresent(arm::setTargetPosition);
        this.getOuttakeSetpoint().ifPresent(intake::setOuttakeSpeed);
        this.getWristExtended().ifPresent(wrist::setExtendedTarget);

        SmartDashboard.putString("State", state.name());
    }
}

enum State {
    LowPickup,
    StowHigh,
    DoubleFeeder,
    LowScore,
    MidScore,
    HighScore,
    StowInFrame,
    StowLow,
    BackwardsMidScore,
    BackwardsHighScore,
    BackwardsLowScore,
    BackwardsDoubleFeeder;

    private Optional<Setpoints> getCubeSetpoints() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = CubeSetpointConstants.kLowPickup; break;
            case StowHigh: result = CubeSetpointConstants.kStowHigh; break;
            case DoubleFeeder: result = CubeSetpointConstants.kDoubleFeeder; break;
            case LowScore: result = CubeSetpointConstants.kLowScore; break;
            case MidScore: result = CubeSetpointConstants.kMidScore; break;
            case HighScore: result = CubeSetpointConstants.kHighScore; break;
            case StowInFrame: result = CubeSetpointConstants.kStowInFrame; break;
            case StowLow: result = CubeSetpointConstants.kStowLow; break;
            case BackwardsMidScore: result = CubeSetpointConstants.kBackwardsMidScore; break;
            case BackwardsHighScore: result = CubeSetpointConstants.kBackwardsHighScore; break;
            case BackwardsLowScore: result = CubeSetpointConstants.kBackwardsLowScore; break;
            case BackwardsDoubleFeeder: result = CubeSetpointConstants.kBackwardsDoubleFeeder; break;
        }

        return Optional.ofNullable(result);
    }

    private Optional<Setpoints> getConeSetpoints() {
        Setpoints result = null;

        switch (this) {
            case LowPickup: result = ConeSetpointConstants.kLowPickup; break;
            case StowHigh: result = ConeSetpointConstants.kStowHigh; break;
            case DoubleFeeder: result = ConeSetpointConstants.kDoubleFeeder; break;
            case LowScore: result = ConeSetpointConstants.kLowScore; break;
            case MidScore: result = ConeSetpointConstants.kMidScore; break;
            case HighScore: result = ConeSetpointConstants.kHighScore; break;
            case StowInFrame: result = ConeSetpointConstants.kStowInFrame; break;
            case StowLow: result = ConeSetpointConstants.kStowLow; break;
            case BackwardsMidScore: result = ConeSetpointConstants.kMidScore; break;
            case BackwardsHighScore: result = ConeSetpointConstants.kHighScore; break;
            case BackwardsLowScore: result = ConeSetpointConstants.kLowScore; break;
            case BackwardsDoubleFeeder: result = ConeSetpointConstants.kDoubleFeeder; break;
        }

        return Optional.ofNullable(result);
    }

    public Optional<Setpoints> getSetpoints() {
        switch (StateManager.getGamepiece()) {
            case Cube: return getCubeSetpoints();
            case Cone: return getConeSetpoints();
            default: return Optional.empty();
        }
    }
}