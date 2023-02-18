package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

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

      public void setPipelineIndex(int index) {
        camera.setPipelineIndex(1);
      }

      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
      NetworkTableEntry tx = table.getEntry("tx");
      NetworkTableEntry ty = table.getEntry("ty");
      NetworkTableEntry ta = table.getEntry("ta");
      
      NetworkTableValue tv_value = table.getValue("tv");
      NetworkTableValue tx_value = table.getValue("tx");
      
      //float tv = table.GetNumber("tv");
      
    @Override
    public void periodic() {
      //read values periodically
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);
      double area = ta.getDouble(0.0);

      float steering_adjust = 0.0f;

        if (tv == 0.0) {
            hasTargets = false;
          } else {
            hasTargets = true;
          }
    }
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
}


