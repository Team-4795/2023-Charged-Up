package frc.robot.subsystems.drive;

import java.util.function.Supplier;

public class GyroIOSim implements GyroIO {
    private double heading = 0.0;
    private double dtheta = 0.0;
    private double offset = 0.0;

    public void addOffset(double change) {
        heading += change;
        dtheta = change / 0.02;
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPositionRad = heading + offset;
        inputs.yawVelocityRadPerSec = dtheta;
    }

    public void reset() {
        heading = 0;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }
}
