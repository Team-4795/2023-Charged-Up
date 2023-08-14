package frc.robot;

import frc.robot.Constants.ConeSetpointConstants;
import frc.robot.Constants.CubeSetpointConstants;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.rollerbar.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.motorizedWrist.*;
import frc.utils.Setpoints;
import org.littletonrobotics.junction.Logger;

public class StateManager extends VirtualSubsystem {
    private static StateManager mInstance;

    public static StateManager getInstance() {
        if (mInstance == null) {
            mInstance = new StateManager();
        }

        return mInstance;
    }

    // What state were in
    private State state;

    // Either what were picking or what were storing
    private Gamepiece gamepiece = Gamepiece.Cube;

    public enum Gamepiece {
        Cube,
        Cone,
    }

    public enum State {
        LowPickup(CubeSetpointConstants.kLowPickup, ConeSetpointConstants.kLowPickup),
        StowHigh(CubeSetpointConstants.kStowHigh, ConeSetpointConstants.kStowHigh),
        DoubleFeeder(CubeSetpointConstants.kDoubleFeeder, ConeSetpointConstants.kDoubleFeeder),
        LowScore(CubeSetpointConstants.kLowScore, ConeSetpointConstants.kLowScore),
        MidScore(CubeSetpointConstants.kMidScore, ConeSetpointConstants.kMidScore),
        HighScore(CubeSetpointConstants.kHighScore, ConeSetpointConstants.kHighScore),
        StowInFrame(CubeSetpointConstants.kStowInFrame, ConeSetpointConstants.kStowInFrame),
        StowLow(CubeSetpointConstants.kStowLow, ConeSetpointConstants.kStowLow),
        BackwardsMidScore(CubeSetpointConstants.kBackwardsMidScore, ConeSetpointConstants.kMidScore),
        BackwardsHighScore(CubeSetpointConstants.kBackwardsHighScore, ConeSetpointConstants.kHighScore),
        BackwardsLowScore(CubeSetpointConstants.kBackwardsLowScore, ConeSetpointConstants.kLowScore),
        BackwardsDoubleFeeder(CubeSetpointConstants.kBackwardsDoubleFeeder, ConeSetpointConstants.kDoubleFeeder),
        BackwardsLowPickup(CubeSetpointConstants.kBackwardsLowPickup, ConeSetpointConstants.kLowPickup),
        BackwardsLowPickupAuto(
                CubeSetpointConstants.kBackwardLowPickupAuto, CubeSetpointConstants.kBackwardLowPickupAuto);

        Setpoints cubeSetpoint;
        Setpoints coneSetpoint;

        State(Setpoints cubeSetpoint, Setpoints coneSetpoint) {
            this.cubeSetpoint = cubeSetpoint;
            this.coneSetpoint = coneSetpoint;
        }
    }

    public Setpoints getSetpoints() {
        switch (getGamepiece()) {
            case Cube:
                return state.cubeSetpoint;
            case Cone:
                return state.coneSetpoint;
            default:
                throw new AssertionError("Invalid gamepiece");
        }
    }

    public StateManager() {
        this.state = State.StowInFrame;
    }

    public void pickCube() {
        gamepiece = Gamepiece.Cube;
        Vision.getInstance().switchToTag();
    }

    public void pickCone() {
        gamepiece = Gamepiece.Cone;
        Vision.getInstance().switchToTape();
    }

    public void setState(State state) {
        this.state = state;
        setSetpoints();
    }

    private boolean isBackwards() {
        return Math.cos(Math.toRadians(Drive.getInstance().getAngle())) < 0;
    }

    public void dpadUp() {
        if (Intake.isStoring()) {
            if (isBackwards()) {
                state = State.BackwardsHighScore;
            } else {
                state = State.HighScore;
            }
        } else {
            if (isBackwards()) {
                state = State.DoubleFeeder;
            } else {
                state = State.BackwardsDoubleFeeder;
            }
        }

        setSetpoints();
    }

    public void dpadLeft() {
        if (Intake.isStoring()) {
            if (isBackwards()) {
                state = State.BackwardsMidScore;
            } else {
                state = State.MidScore;
            }
        } else {
            state = State.LowPickup;
        }

        setSetpoints();
    }

    public void dpadDown() {
        if (Intake.isStoring()) {
            if (isBackwards()) {
                state = State.BackwardsLowScore;
            } else {
                state = State.LowScore;
            }
        } else {
            state = State.BackwardsLowPickup;
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

    public Double getArmSetpoint() {
        return getSetpoints().arm;
    }

    public Double getOuttakeSetpoint() {
        return getSetpoints().outtake;
    }

    public Boolean getWristExtended() {
        return getSetpoints().wrist;
    }

    public Boolean getRollerbarExtended() {
        return getSetpoints().rollerbar;
    }

    public State getState() {
        return this.state;
    }

    public boolean isStowing() {
        switch (this.state) {
            case StowInFrame:
                return true;
            default:
                return false;
        }
    }

    public Gamepiece getGamepiece() {
        return gamepiece;
    }

    public void setSetpoints() {
        Arm.getInstance().setTargetPosition(getArmSetpoint());
        Intake.getInstance().setOuttakeSpeed(getOuttakeSetpoint());
        Wrist.getInstance().setExtendedTarget(getWristExtended());
        Rollerbar.getInstance().setExtendedTarget(getRollerbarExtended());
    }

    public void periodic() {
        Logger.getInstance().recordOutput("StateManager/Gamepiece", gamepiece.toString());
        Logger.getInstance().recordOutput("StateManager/State", state.toString());
    }
}
