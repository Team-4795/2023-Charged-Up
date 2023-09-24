package frc.robot.subsystems.motorizedWrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    
    @AutoLog
    public static class WristIOInputs {
        public double motorVolts = 0;
        public double angle = 0; //in revolutions
        public double angularVelocity = 0;
        public double current = 0;
    }

    public default void updateInputs(WristIOInputs inputs){}

    public default void set(double motorSpeed){}
}
