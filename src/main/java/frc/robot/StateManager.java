package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import frc.utils.Gamepiece;
import frc.utils.Setpoints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CubeSetpointConstants;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.ConeSetpointConstants;

public class StateManager {
    private LiftArm arm;
    private EndEffectorIntake intake;
    private Wrist wrist;

    // What state were in
    private State state;

    public enum State {
        LowPickup,
        SingleFeeder,
        DoubleFeeder,
        LowScore,
        MidScore,
        HighScore,
        StowInFrame;
            
        private Setpoints getCubeSetpoints() {
            switch (this) {
                case LowPickup: return CubeSetpointConstants.kLowPickup;
                case SingleFeeder: return CubeSetpointConstants.kSingleFeeder;
                case DoubleFeeder: return CubeSetpointConstants.kDoubleFeeder;
                case LowScore: return CubeSetpointConstants.kLowScore;
                case MidScore: return CubeSetpointConstants.kMidScore;
                case HighScore: return CubeSetpointConstants.kHighScore;
                case StowInFrame: return CubeSetpointConstants.kStowInFrame;
                default: return null;
            }
        }
    
        private Setpoints getConeSetpoints() {
            switch (this) {
                case LowPickup: return ConeSetpointConstants.kLowPickup;
                case SingleFeeder: return ConeSetpointConstants.kSingleFeeder;
                case DoubleFeeder: return ConeSetpointConstants.kDoubleFeeder;
                case LowScore: return ConeSetpointConstants.kLowScore;
                case MidScore: return ConeSetpointConstants.kMidScore;
                case HighScore: return ConeSetpointConstants.kHighScore;
                case StowInFrame: return ConeSetpointConstants.kStowInFrame;
                default: return null;
            }
        }
    
        public Optional<Setpoints> getSetpoints() {
            switch (Gamepiece.getGamepiece()) {
                case Cube: return Optional.ofNullable(getCubeSetpoints());
                case Cone: return Optional.ofNullable(getConeSetpoints());
                default: return Optional.empty();
            }
        }
    }

    public StateManager(LiftArm arm, EndEffectorIntake intake, Wrist wrist) {
        this.state = State.StowInFrame;
        this.arm = arm;
        this.intake = intake;
        this.wrist = wrist;
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

    private void setSetpoints() {
        this.getArmSetpoint().ifPresent(arm::setPosition);
        this.getOuttakeSetpoint().ifPresent(intake::setOuttakeSpeed);
        this.getWristExtended().ifPresent(wrist::setExtendedTarget);

        SmartDashboard.putString("State", state.name());
    }
}