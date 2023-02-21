// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.WPIUtilJNI;
import frc.utils.SwerveUtils;


public class DriveSubsystem extends SubsystemBase {
  //Create field2d
  private final Field2d m_field = new Field2d();
  
  // Create MAXSwerveModules
  
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor

  AHRS m_gyro = new AHRS(SPI.Port.kMXP);


  private double[][] rotation = new double[3][3];
  private double[] rotationChanges = new double[3];
  private double balanceSpeed = 0.0;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle() + Constants.DriveConstants.kChassisAngularOffset),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    //SmartDashboard.putNumber("AngleYaw", m_gyro.getYaw());
    //SmartDashboard.putNumber("Angle", m_gyro.getAngle());
    //SmartDashboard.putBoolean("isconnected", m_gyro.isConnected());
    //SmartDashboard.putBoolean("iscalibrating", m_gyro.isCalibrating());

    

    m_odometry.update(
      Rotation2d.fromDegrees(-m_gyro.getAngle()+ Constants.DriveConstants.kChassisAngularOffset),
        
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
            
        });

    SmartDashboard.putNumber("Angle of Elevation", getElevationAngle());
    SmartDashboard.putNumber("Elevation velocity", getElevationVelocity());
    SmartDashboard.putNumber("Angle of Elevation (w/ Matrix)", getElevationAngleV2());
    SmartDashboard.putNumber("Elevation velocity (w/ Matrix)", getElevationVelocityV2());
    SmartDashboard.putNumber("Balancing Speed", getBalanceSpeed());
    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(-m_gyro.getAngle()+ Constants.DriveConstants.kChassisAngularOffset),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }


      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()+ Constants.DriveConstants.kChassisAngularOffset))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void setBalanceSpeed(double value){
    balanceSpeed = value;
  }
  //pick up these changes please
  public double getBalanceSpeed(){
    return balanceSpeed;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()+ Constants.DriveConstants.kChassisAngularOffset);
  }

  //angle between xy-plane and the forward vector of the drivebase - potentially doesn't work
  public double getElevationAngle(){
    return Rotation2d.fromDegrees(m_gyro.getPitch()).getDegrees();
  }

  public double getElevationVelocity(){
    return m_gyro.getRawGyroX();
  }

  public void setRotationMatrix(double xOffset, double yOffset, double zOffset){
    double x = m_gyro.getPitch() * Math.PI / 180 + xOffset;
    double y = m_gyro.getRoll() * Math.PI / 180 + yOffset; //is this x or y axis???^ description some garbage bruh
    double z = m_gyro.getYaw() * Math.PI / 180 + zOffset;
    rotation[0] = new double[] {Math.cos(y) * Math.cos(z), Math.sin(x) * Math.sin(y) * Math.cos(z) - Math.cos(x) * Math.sin(z), Math.cos(x) * Math.sin(y) * Math.cos(z) - Math.sin(x) * Math.sin(z)};
    rotation[1] = new double[] {Math.cos(y) * Math.cos(z), Math.sin(x) * Math.sin(y) * Math.sin(z) + Math.cos(x) * Math.cos(z), Math.cos(x) * Math.sin(y) * Math.sin(z) - Math.sin(x) * Math.cos(z)};
    rotation[2] = new double[] {-Math.sin(y), Math.sin(x) * Math.cos(y), Math.cos(x) * Math.cos(y)};
  }

  public double[] multiplyRotationMatrix(double[] vector){
    //vector is a three by 1 matrix (array)
    double sum;
    double[] result = new double[3];
    for(int i = 0; i < 3; i++){
      sum = 0;
      for(int j = 0; j < 3; j++){
        sum += rotation[i][j] * vector[j];
      }
      result[i] = sum;
    }
    return result;
  }

  //should not work theoretically, because it's not as simple as putting in an xy vector -headingAngle from the [0,1,0] vector
  public double getElevationAngleV2(){
    setRotationMatrix(0, 0, 0);
    double headingAngle = getHeading().getRadians();
    double[] outputVector = multiplyRotationMatrix(new double[] {-Math.sin(headingAngle), Math.cos(headingAngle), 0});
    return Math.atan2(outputVector[2], (Math.sqrt(Math.pow(outputVector[0], 2) + Math.pow(outputVector[1], 2))));
  }

  public double getElevationVelocityV2(){
    setRotationMatrix(0, 0, 0);
    double headingAngle = getHeading().getRadians();
    double[] hold = multiplyRotationMatrix(new double[] {-Math.sin(headingAngle), Math.cos(headingAngle), 0});
    double original = Math.atan2(hold[2], (Math.sqrt(Math.pow(hold[0], 2) + Math.pow(hold[1], 2))));
    setRotationMatrix(0.0001, 0, 0);
    hold = multiplyRotationMatrix(new double[] {-Math.sin(headingAngle), Math.cos(headingAngle), 0});
    double xChange = Math.atan2(hold[2], (Math.sqrt(Math.pow(hold[0], 2) + Math.pow(hold[1], 2))));
    setRotationMatrix(0, 0.0001, 0);
    hold = multiplyRotationMatrix(new double[] {-Math.sin(headingAngle), Math.cos(headingAngle), 0});
    double yChange = Math.atan2(hold[2], (Math.sqrt(Math.pow(hold[0], 2) + Math.pow(hold[1], 2))));
    setRotationMatrix(0, 0, 0.0001);
    hold = multiplyRotationMatrix(new double[] {-Math.sin(headingAngle + 0.0001), Math.cos(headingAngle + 0.0001), 0});
    double zChange = Math.atan2(hold[2], (Math.sqrt(Math.pow(hold[0], 2) + Math.pow(hold[1], 2))));
    rotationChanges[0] = (xChange - original) / 0.0001;
    rotationChanges[1] = (yChange - original) / 0.0001;
    rotationChanges[2] = (zChange - original) / 0.0001;
    return (rotationChanges[0] * m_gyro.getRawGyroX() + rotationChanges[1] * m_gyro.getRawGyroY() + rotationChanges[2] * m_gyro.getRawGyroZ());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setBreakMode()
  {
    m_frontLeft.setBreakMode();
    m_frontRight.setBreakMode();
    m_rearLeft.setBreakMode();
    m_rearRight.setBreakMode();
  }
}
