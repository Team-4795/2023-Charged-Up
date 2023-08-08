package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class LimeLight extends SubsystemBase{
    
    private NetworkTable camera;

    double camX;
    double camY;
    double camArea;

    boolean hasTargets;

    public static LimeLight mInstance;

    public static LimeLight getInstance(){
        if(mInstance == null){
            mInstance = new LimeLight();
        }
        return mInstance;
    }

    private LimeLight(){
        camera = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getX(){
        return camX;
    }

    public double getY(){
        return camY;
    }

    public double camArea(){
        return camArea;
    }

    @Override
    public void periodic(){
        camX = camera.getEntry("tx").getDouble(VisionConstants.nTableDefault);
        camY = camera.getEntry("ty").getDouble(VisionConstants.nTableDefault);
        camArea = camera.getEntry("ta").getDouble(VisionConstants.nTableDefault);
        // hasTargets = (camera.getEntry("tv").getBoolean(false));
        // I feel like the above should work^ but the API says to use getDouble instead
        hasTargets = (camera.getEntry("tv").getDouble(VisionConstants.nTableDefault) == 1); 
        Logger.getInstance().recordOutput("Limelight Area", camArea);
    }

}
