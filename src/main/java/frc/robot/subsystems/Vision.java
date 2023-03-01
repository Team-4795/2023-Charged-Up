package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;


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
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
      NetworkTableEntry tx = table.getEntry("tx");
      NetworkTableEntry ty = table.getEntry("ty");
      NetworkTableEntry ta = table.getEntry("ta");
      NetworkTableEntry tv = table.getEntry("tv");

      NetworkTableValue tv_value = table.getValue("tv");
      NetworkTableValue tx_value = table.getValue("tx");
      double area;
      double y;
      double x;
      NetworkTableEntry botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose"); //AHHHHHHH
            
    @Override
    public void periodic() {
            //read values periodically
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);
      double area = ta.getDouble(0.0);

      double targetAngle = x; //need to find out if tx is angle or distance
      float steering_adjust = 0.0f;

        if (tv.getDouble(0.0) == 0.0) {
            hasTargets = false;
            // range = -1;
          } else {
            // range = PhotonUtils.calculateDistanceToTargetMeters(
            //         CameraHeight,
            //         TargetHeight,
            //         cameraPitchRadians ,
            //         Units.degreesToRadians(result.getBestTarget().getPitch()));
            hasTargets = true;
    }

    // SmartDashboard.putNumber("Distance between target", range);   
    SmartDashboard.putBoolean("Has target", hasTargets);   
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

          /* 
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Vision");
      builder.addBooleanProperty("Has target", () -> hasTargets, null);
      builder.addDoubleProperty("Goal angle", () -> targetAngle, null);
            //post to smart dashboard periodically
      SmartDashboard.putNumber("LimelightX", x);
      SmartDashboard.putNumber("LimelightY", y);
      SmartDashboard.putNumber("LimelightArea", area);
    }
    */

}
}




