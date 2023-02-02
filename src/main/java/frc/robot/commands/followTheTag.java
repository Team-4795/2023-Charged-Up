package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveSubsystem;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagDetector;

public class followTheTag implements Runnable {
  private final UsbCamera camera;
  private final CvSink cvSink;
  private final CvSource outputStream;
  private final AprilTagDetector detector;
  private final DriveSubsystem drivetrain;

  public followTheTag(UsbCamera camera, Drivetrain drivetrain) {
    this.camera = camera;
    this.drivetrain = drivetrain;

    cvSink = CameraServer.getInstance().getVideo(camera);
    outputStream = CameraServer.getInstance().putVideo("Tag Detection", 640, 480);

    detector = new AprilTagDetector();
  }

  @Override
  public void run() {
    Mat inputImage = new Mat();
    Mat outputImage = new Mat();

    while (!Thread.interrupted()) {
      cvSink.grabFrame(inputImage);

      List<AprilTagDetection> detections = detector.detectAprilTags(inputImage);
      if (!detections.isEmpty()) {
        Pose2d robotPose = drivetrain.getPose();

        List<Pose2d> tagPoses = new ArrayList<>();
        for (AprilTagDetection detection : detections) {
          double tagX = Units.inchesToMeters(detection.getRelativeTranslation().getX());
          double tagY = Units.inchesToMeters(detection.getRelativeTranslation().getY());
          double tagTheta = detection.getRelativeRotation().getRadians();

          Pose2d tagPose = new Pose2d(tagX, tagY, new Rotation2d(tagTheta));
          tagPoses.add(tagPose);

          Imgproc.drawContours(outputImage, detection.getContours(), -1, new Scalar(0, 255, 0));
        }

        SmartDashboard.putNumber("Tag Count", tagPoses.size());

        NetworkTableInstance.getDefault().getTable("Vision").getEntry("Tag Count");
