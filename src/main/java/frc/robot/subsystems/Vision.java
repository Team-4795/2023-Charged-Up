package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;


public class Vision extends SubsystemBase{
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.kSnakeEyesCamera);
    final double CameraHeight = VisionConstants.kCameraHeight;
    final double TargetHeight = VisionConstants.kTargetHeight;
    final double cameraPitchRadians = VisionConstants.kCameraPitchRadians;
    public boolean hasTargets = false;
    public boolean isTargeting = true;
   
    private double targetAngle = VisionConstants.kTargetAngle;
    private double range;
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

      public void setLEDBrightness() {
        //set led brightness here
      }

      public void pipelineIndex(int index) {
        camera.setPipelineIndex(index);
      }

      public void switchToTag() {
        pipelineIndex(0);
        disableLED();
      }
    
      public void switchToTape() {
        pipelineIndex(1);
        enableLED();
      }

      public void targetingLED() {
        if (isTargeting == false) {
          disableLED();
        }
        else {
          enableLED();
        }
      }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        camera.setDriverMode(false);
        if (result.hasTargets()) {
            hasTargets = true;
            targetAngle = result.getBestTarget().getPitch(); //pitch or yaw?
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    CameraHeight,
                    TargetHeight,
                    cameraPitchRadians ,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
          } else {
            hasTargets = false;
            range = -1;
            //targetAngle = -1;
          }
        SmartDashboard.putNumber("Distance between target", range);   
    }
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Vision");
      builder.addBooleanProperty("Has target", () -> hasTargets, null);
      builder.addDoubleProperty("Goal angle", () -> targetAngle, null);
    }


}


