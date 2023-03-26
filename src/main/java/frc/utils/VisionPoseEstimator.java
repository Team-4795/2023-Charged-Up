package frc.utils;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;

public class VisionPoseEstimator {
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private AprilTagFieldLayout field;
    private Optional<EstimatedRobotPose> result;
    private Pose2d estimate;

    public static List<AprilTag> Heho = new ArrayList<>();

    public VisionPoseEstimator(String camera){
        Heho.add(new AprilTag(6, new Pose3d(new Translation3d(1.02743, 4.424426, 0.462788), new Rotation3d(new Quaternion(1, 0, 0, 0)))));
        Heho.add(new AprilTag(7, new Pose3d(new Translation3d(1.02743, 2.748026, 0.462788), new Rotation3d(new Quaternion(1, 0, 0, 0)))));
        Heho.add(new AprilTag(8, new Pose3d(new Translation3d(1.02743, 1.071626, 0.462788), new Rotation3d(new Quaternion(1, 0, 0, 0)))));

        this.camera = new PhotonCamera(camera);
        try {
            field = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            //field = new AprilTagFieldLayout(Heho, 0, 8.013);
        } catch (IOException e) {
            DriverStation.reportError("!!!Apriltags be Stalin'!!!", e.getStackTrace());
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
