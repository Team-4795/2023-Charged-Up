package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;


public class Vision extends SubsystemBase{
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.SnakeEyesCamera);
    final double CameraHeight = VisionConstants.CameraHeight;
    final double TargetHeight = VisionConstants.TargetHeight;
    final double cameraPitchRadians = VisionConstants.cameraPitchRadians;
    public boolean hasTargets = false;
    private double targetAngle = 0;
    double forwardSpeed;
    double x_pitch = 0;

      public boolean hasTargets() {
        return hasTargets;
      }

      public double getTargetAngle() {
        return targetAngle;
      }

      public void enableLED() {
        camera.setLED(VisionLEDMode.kOn);
      }

      public void disableLED() {
        camera.setLED(VisionLEDMode.kOff);
      }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            hasTargets = true;
            targetAngle = result.getBestTarget().getPitch(); //pitch or yaw?
          } else {
            hasTargets = false;
//            targetAngle = -1;
          }
    }
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Vision");
      builder.addBooleanProperty("Has target", () -> hasTargets, null);
      builder.addDoubleProperty("Goal angle", () -> targetAngle, null);
    }
}