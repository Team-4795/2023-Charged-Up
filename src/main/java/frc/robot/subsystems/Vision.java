package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Vision extends SubsystemBase{
    private final PhotonCamera camera = new PhotonCamera(Constants.SnakeEyesCamera);
    //Will change camera name later
    final double CameraHeight = Constants.CameraHeight;
    final double TargetHeight = Constants.TargetHeight;
    final double cameraPitchRadians = Constants.cameraPitchRadians;
    public boolean hasTargets = false;

    public boolean visibleTargets()
    {
        return hasTargets;
    }

    public void EnableLED() 
    {
        camera.setLED(VisionLEDMode.kOn);
    }

    public void DisableLED()
    {
        camera.setLED(VisionLEDMode.kOff);
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();

        if(result.hasTargets())
        {
            hasTargets = true;
            double range = 
                PhotonUtils.calculateDistanceToTargetMeters(CameraHeight, 
                TargetHeight, 
                cameraPitchRadians, 
                Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        else
        {
            hasTargets = false;
        }
    }
}
