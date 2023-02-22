// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.lang.annotation.Target;




public class Align extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final Vision vision;
  private boolean isAligned;
  private long alignStart; 
  private PIDController rotationPID;
  private final PhotonCamera camera;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  
  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS); //needs adjusting 
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;

  public Align(DriveSubsystem driveSubsystem, Vision vision, PhotonCamera camera) {
    this.driveSubsystem = driveSubsystem;
    this.vision = vision;
    this.camera = camera;
    rotationPID = new PIDController(0.1, 0, 0);

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    isAligned = false;
    alignStart = System.currentTimeMillis();

    lastTarget = null;
    var robotPose = driveSubsystem.getPose();

    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    camera.setPipelineIndex(0); //placeholder

  }

  @Override
  public void execute() {
    /* This was an earlier attempt at turning 180 degrees, just keeping it in to see if i was on a right track with it
    double error = driveSubsystem.getHeading();
    double rotation = rotationPID.calculate(error);
    driveSubsystem.drive(0, 0, rotation, true); //is it going to be true or false?
     */


    var photonRes = camera.getLatestResult();
    var robotPose2d = driveSubsystem.getPose();
    var robotPose = 
    new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0, 
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
        if (photonRes.hasTargets()) {
          // Find the tag we want to chase
          var targetOpt = photonRes.getTargets().stream()
              .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
              .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
              .findFirst();
          if (targetOpt.isPresent()) {
            var target = targetOpt.get();
            // This is new target data, so recalculate the goal
            lastTarget = target;
            
            // Transform the robot's pose to find the camera's pose
            var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);
    
            // Trasnform the camera's pose to the target's pose
            var camToTarget = target.getBestCameraToTarget();
            var targetPose = cameraPose.transformBy(camToTarget);
            
            // Transform the tag's pose to set our goal
            var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
    
            // Drive
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
          }
        }
        
        if (lastTarget == null) {
          // No target has been visible
          driveSubsystem.drive(0,0,0,true,true);
        } else {
          // Drive to the target
          var xSpeed = xController.calculate(robotPose.getX());
          if (xController.atGoal()) {
            xSpeed = 0;
          }
    
          var ySpeed = yController.calculate(robotPose.getY());
          if (yController.atGoal()) {
            ySpeed = 0;
          }
    
          var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
          if (omegaController.atGoal()) {
            omegaSpeed = 0;
          }
          driveSubsystem.drive(xSpeed, ySpeed, omegaSpeed, true, true);
        }
      }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAligned && System.currentTimeMillis() - alignStart > 200;
  }
  public void end(boolean interrupted) {
    driveSubsystem.drive(0,0,0, true,true);
  }


}