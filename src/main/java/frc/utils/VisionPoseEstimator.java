package frc.utils;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;

public class VisionPoseEstimator {
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private AprilTagFieldLayout field;
    private Optional<EstimatedRobotPose> result;
    private Pose2d estimate;

    public VisionPoseEstimator(String camera){
        this.camera = new PhotonCamera(camera);
        try {
            field = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            DriverStation.reportError("Apriltags be Stalin'", e.getStackTrace());
        }
        estimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP, this.camera, VisionConstants.ROBOT_TO_CAMERA);
    }

    public void estimateRobotPose(Pose2d reference){
        estimator.setReferencePose(reference);
        result = estimator.update();
        result.ifPresent(new Consumer<EstimatedRobotPose>() {
            @Override
            public void accept(EstimatedRobotPose pose) {
                estimate = pose.estimatedPose.toPose2d();
            }
        });
    }

    public Pose2d getPoseEstimate(){
        return estimate;
    }
    
}
