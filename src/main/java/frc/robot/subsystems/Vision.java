package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
//import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;


public class Vision extends SubsystemBase{
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.kSnakeEyesCamera);
    final double CameraHeight = VisionConstants.kCameraHeight;
    final double TargetHeight = VisionConstants.kTargetHeight;
    final double cameraPitchRadians = VisionConstants.kCameraPitchRadians;
    public boolean hasTargets = false;
    private double targetAngle = VisionConstants.kTargetAngle;
    double forwardSpeed;
    double x_pitch = VisionConstants.kX_Pitch;

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

      public void setPipelineIndex(int index){
        camera.setPipelineIndex(index);
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