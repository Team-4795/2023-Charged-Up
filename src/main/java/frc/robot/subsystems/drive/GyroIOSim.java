package frc.robot.subsystems.drive;

public class GyroIOSim implements GyroIO {
    private double heading = 0.0;
    private double offset = 0.0;

    public void addOffset(double change) {
        heading += change;
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPositionRad = heading + offset;
    }

    public void reset() {
        heading = 0;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }
}
