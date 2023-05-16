package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double pressure = 0.0;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void set(Value value) {};
}
