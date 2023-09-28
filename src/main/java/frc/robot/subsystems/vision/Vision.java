package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {

  private NetworkTable camera;

  //because camera is mounted sideways, camX = ty and camY = tx;
  private double camX;
  private double camY; 
  private double camArea;
  private double[] botpose;
  private double[] loggedpose;
  private boolean hasTargets;

  private static Vision mInstance;

  public static Vision getInstance(){
    if(mInstance == null){
        mInstance = new Vision();
    }
    return mInstance;
  }

  private Vision(){
    camera = NetworkTableInstance.getDefault().getTable("limelight");
    loggedpose = new double[7];
  }

  public boolean hasTargets() {
    return hasTargets;
  }

  public double getAngleY() {
    return camY;
  }

  public double getAngleX() {
    return -camX;
  }

  public double getArea() {
    return camArea;
  }

  public void pipelineIndex(int index) {
    camera.getEntry("pipeline").setNumber(index);
  }

  public void switchToTag() {
    pipelineIndex(0);
  }

  public void switchToTape() {
    pipelineIndex(1);
  }

  @Override
  public void periodic() {
    camX = camera.getEntry("ty").getDouble(0.0);
    camY = camera.getEntry("tx").getDouble(0.0);
    camArea = camera.getEntry("ta").getDouble(0.0);
    hasTargets = (camera.getEntry("tv").getDouble(0.0) == 1);
    botpose = camera.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    Rotation3d botRotation = new Rotation3d(botpose[4], botpose[3], botpose[5]);
    Quaternion botQuaternion = botRotation.getQuaternion();
    loggedpose[0] = botpose[0];
    loggedpose[1] = botpose[1];
    loggedpose[2] = botpose[2];
    loggedpose[3] = botQuaternion.getW();
    loggedpose[4] = botQuaternion.getX();
    loggedpose[5] = botQuaternion.getY();
    loggedpose[6] = botQuaternion.getZ();


    SmartDashboard.putBoolean("Vision Target?", hasTargets);
    SmartDashboard.putNumber("Target Area", camArea);
    SmartDashboard.putNumber("Displacement Angle X", camX);
    SmartDashboard.putNumber("Displacement Angle Y", camY);
    SmartDashboard.putNumberArray("Botpose", loggedpose);
  }

}
