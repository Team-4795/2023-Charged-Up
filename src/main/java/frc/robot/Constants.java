package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.7; // radians per second
    public static final double kMagnitudeSlewRate = 2.3; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.5; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.75);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset  = Math.PI / 2;

    //Chassis Angular Offset

    public static final double kChassisAngularOffset = -90;


    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 9;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 60; // amps
    public static final int kTurningMotorCurrentLimit = 25; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kArmDeadband = 0.05;
    public static final double kArmManualSpeed = 0.01;
  }

  public static final class ControlContants {
    public static final int kArmUpAxis = 3;
    public static final int kArmDownAxis = 2;

    public static final int kSetxButton = 1;
    public static final int kResetHeadingButton = 2;
    public static final int kTapeAlignButton = 3;

    public static final int kReverseIntakePOV = 90;
    public static final int kIntakePOV = 270;

    public static final int kPickConeButton = 5;
    public static final int kPickCubeButton = 6;

    public static final int kExtendButton = 3;
    public static final int kRetractButton = 4;

    public static final int kIsStoringButton = 2;
    public static final int kIsNotStoringButton = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class RotationConstants {
    public static final double kP = .01;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kMinimumAngle = -180;
    public static final double kMaximumAngle = 180;
  }

  public static final class TapeAlignConstants {
    public static final double kP = .011;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kAngularThreshold = 2;
    public static final double kRotationSetpoint = 0;
    public static final double kTranslationSetpoint = 0;
  }

  public static final class ArmConstants {
    public static final int kLeftArmMotorCANID = 10;
    public static final int kRightArmMotorCANID = 11;
    public static final double kP = 1.6;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;
    public static final double kMinOutput = -0.3;
    public static final double kMaxOutput = 0.3;
    public static final double kRampRate = 0.1;
    public static final double kConversionFactor = 1;
    public static final int kCurrentLimit = 60;
    public static final double kGearing = 72;
    public static final boolean kEnableForwardLimit = false;
    public static final boolean kEnableReverseLimit = false;
    public static final double kForwardLimit = 0.95;
    public static final double kReverseLimit = 10;
    public static final double kLowWristLimit = 0.2;
    public static final double kHighWristLimit = 0.8;
    public static final double kLowSetpointLimit = 0.12;
    public static final double kHighSetpointLimit = 0.97;
  }

  public static final class IntakeConstants{
    public static final double kIntakeSpeed = 1;
    public static final double kOuttakeSpeed = -0.3;
    public static final double kSlowCubeIntakeSpeed = 0.5;
    public static final double kSlowConeIntakeSpeed = 0.5;
    public static final double kOtherIntakeSpeed = 0.25;
    public static final double kRequestedSpeed = 0.25;
    public static final int kIntakeCANID = 24;
    public static final int kPHCANID = 1;
    public static final double kMaxPressure = 120;
    public static final double kMinPressure = 90;
    public static final int kHiLetGoPort = 0;
    public static final int kForwardChannel = 1;
    public static final int kReverseChannel = 2;
    public static final int kCurrentLimit = 25;

  }

  public static final class CubeSetpointConstants{
    public static final double kLowPickupArm = 0.933;
    public static final double kSingleFeederArm = 0.7;
    public static final double kDoubleFeederArm = 0.64;
    public static final double kLowScoreArm = 0.91;
    public static final double kMidScoreArm = 0.7;
    public static final double kHighScoreArm = 0.64;
    public static final double kStowInFrameArm = 0.16;
    public static final double kStowLowArm = 0.96;
    
    public static final boolean kLowPickupWrist = false;
    public static final boolean kSingleFeederWrist = false;
    public static final boolean kDoubleFeederWrist = true;
    public static final boolean kLowScoreWrist = false;
    public static final boolean kMidScoreWrist = true;
    public static final boolean kHighScoreWrist = true;
    public static final boolean kStowInFrameWrist = false;
    public static final boolean kStowLowWrist = false;

    public static final double kLowPickupIntake = 0.5;
    public static final double kSingleFeederIntake = 0.5;
    public static final double kDoubleFeederIntake = 0.5;
    public static final double kLowScoreIntake = 0.1;
    public static final double kMidScoreIntake = 0.1;
    public static final double kHighScoreIntake = 0.1;
    public static final double kStowInFrameIntake = 0.1;
    public static final double kStowLowIntake = 0.1;

  }

  public static final class ConeSetpointConstants{
    public static final double kLowPickupArm = 0.955;
    public static final double kSingleFeederArm = 0.6;
    public static final double kDoubleFeederArm = 0.64;
    public static final double kLowScoreArm = 0.87;
    public static final double kMidScoreArm = 0.73;
    public static final double kHighScoreArm = 0.64;
    public static final double kStowInFrameArm = 0.16;
    public static final double kStowLowArm = 0.96;
    
    public static final boolean kLowPickupWrist = false;
    public static final boolean kSingleFeederWrist = false;
    public static final boolean kDoubleFeederWrist = true;
    public static final boolean kLowScoreWrist = false;
    public static final boolean kMidScoreWrist = false;
    public static final boolean kHighScoreWrist = false;
    public static final boolean kStowInFrameWrist = false;
    public static final boolean kStowLowWrist = false;

    public static final double kLowPickupIntake = 1.0;
    public static final double kSingleFeederIntake = 1.0;
    public static final double kDoubleFeederIntake = 1.0;
    public static final double kLowScoreIntake = 0.1;
    public static final double kMidScoreIntake = 0.1;
    public static final double kHighScoreIntake = 0.1;
    public static final double kStowInFrameIntake = 0.1;
    public static final double kStowLowIntake = 0.1;

  }
  public static class VisionConstants {
    public static final String kSnakeEyesCamera = "OV5647";
    public static final double kCameraHeight = 21;
    public static final double kTargetHeight = 0;
    public static final double kCameraPitchRadians = -5;
    public static final double kX_Pitch = 0;
    public static final double kTargetAngle = 0;

    /**
     * Physical location of the camera on the robot, relative to the center of the robot.
     */
    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
  }

}
