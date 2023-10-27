package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;

public class Vision extends SubsystemBase {

  private NetworkTable camera;

  //because camera is mounted sideways, camX = ty and camY = tx;
  private double camX;
  private double camY; 
  private double camArea;
  private double dist;
  public double[] botpose;
  public double[] loggedpose;
  public Rotation3d botRotation;
  private boolean hasTargets;

  private LimelightResults llresults;

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
    botRotation = new Rotation3d();
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
    //camera.getEntry("pipeline").setNumber(index);
    LimelightHelpers.setPipelineIndex("", index);
  }

  public void switchToTag() {
    pipelineIndex(0);
  }

  public void switchToTape() {
    pipelineIndex(1);
  }

  public double getDistance() {
    return dist;
  }

  @Override
  public void periodic() {
    llresults = LimelightHelpers.getLatestResults("");
    int numAprilTags = llresults.targetingResults.targets_Fiducials.length;

    camX = LimelightHelpers.getTX("");
    camY = LimelightHelpers.getTY("");
    camArea = LimelightHelpers.getTA("");
    botpose = LimelightHelpers.getBotPose_wpiRed("");
    dist = LimelightHelpers.getTargetPose3d_CameraSpace("").getTranslation().getZ();
    hasTargets = LimelightHelpers.getTV("");

    //camX = camera.getEntry("ty").getDouble(0.0);
    //camY = camera.getEntry("tx").getDouble(0.0);
    //camArea = camera.getEntry("ta").getDouble(0.0);
    //hasTargets = (camera.getEntry("tv").getDouble(0.0) == 1);
    //botpose = camera.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    
    botRotation = new Rotation3d(botpose[3]*Math.PI/180, botpose[4]*Math.PI/180, botpose[5]*Math.PI/180);
    Quaternion botQuaternion = botRotation.getQuaternion();

    loggedpose[0] = botpose[0];
    loggedpose[1] = botpose[1];
    loggedpose[2] = botpose[2];
    loggedpose[3] = botQuaternion.getW();
    loggedpose[4] = botQuaternion.getX()*3.14159/180;
    loggedpose[5] = botQuaternion.getY();
    loggedpose[6] = botQuaternion.getZ();

    Logger.getInstance().recordOutput("Vision/Distance", dist);
    SmartDashboard.putBoolean("Vision Target?", hasTargets);
    SmartDashboard.putNumber("Target Area", camArea);
    SmartDashboard.putNumber("Displacement Angle X", camX);
    SmartDashboard.putNumber("Displacement Angle Y", camY);
    SmartDashboard.putNumber("Number of AprilTags", numAprilTags);
    SmartDashboard.putNumberArray("Botpose", loggedpose);

  }

}
