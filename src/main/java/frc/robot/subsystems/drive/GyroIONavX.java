package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO {
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public void updateInputs(GyroIOInputs inputs) {
        inputs.rollPositionRad = gyro.getRoll();
        inputs.pitchPositionRad = gyro.getPitch();
        inputs.yawPositionRad = gyro.getAngle();
    }

    public void reset() {
        gyro.reset();
    }

    public void setOffset(double offset) {
        gyro.setAngleAdjustment(offset);
    }
}
