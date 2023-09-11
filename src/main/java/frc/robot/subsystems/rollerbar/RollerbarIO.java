package frc.robot.subsystems.rollerbar;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface RollerbarIO {
    @AutoLog
    public static class RollerbarIOInputs {
        public double appliedOutput = 0.0;
        public double currentAmps = 0.0;
        public double pressure = 0.0;
    }

    public default void updateInputs(RollerbarIOInputs inputs) {}

    public default void setExtension(Value value) {};

    public default Value getExtension() {
        return Value.kReverse;
    };

    public default void setSpin(double speed) {};
}
