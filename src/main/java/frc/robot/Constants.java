package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.utils.Setpoints;
import java.util.HashMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DT = 0.02;

    public static enum RobotType {
        Comp,
        Sim,
        Replay
    }

    public static final RobotType robot = RobotType.Sim;

    public static RobotType getRobot() {
        return robot;
    }


    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 10.0; // radians per second
        public static final double kMagnitudeSlewRate = 10.0; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 10.0; // percent per second (1 = 100%)

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
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // Chassis Angular Offset

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
        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kWheelDiameterMeters = Units.inchesToMeters(2.9);
        public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    
        public static final int kDrivingMotorPinionTeeth = 14;
        public static final double kDrivingMotorReduction = (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0);
    
        public static final double kDrivingEncoderPositionFactor = 1.0 / kDrivingMotorReduction; // radians
        public static final double kDrivingEncoderVelocityFactor = (1.0 / kDrivingMotorReduction) / 60.0; // radians per second
    
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    
        public static final int kDrivingMotorCurrentLimit = 60; // amps
        public static final int kTurningMotorCurrentLimit = 25; // amps
      }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final CommandXboxController driverController =
                new CommandXboxController(OIConstants.kDriverControllerPort);
        public static final CommandXboxController operatorController =
                new CommandXboxController(OIConstants.kOperatorControllerPort);

        public static final double kDriveDeadband = 0.1;
        public static final double kArmDeadband = 0.05;
        public static final double kArmManualSpeed = 0.01;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final PIDController AutoXcontroller = new PIDController(2, 0, 0);
        public static final PIDController AutoYcontroller = new PIDController(4, 0, 0);
        public static final PIDController AutoRotationcontroller = new PIDController(3.0, 0, 0);

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
        public static final double VisionXspeed = 0;
        public static final double VisionYspeed = 0;
        public static final double VisionMoveFastX = 0;
        public static final double VisionMoveFastY = .3;

        public static final double kIntakeDelay = 0.25;
        public static final double kIntakeWaitTime = 0.2;
        public static final double kOuttakeDelay = 0.25;

        public static final double toZeroBound = 0.000001;

        public static final double platformMaxAngle = 10;

        // constant speed during command
        public static final double balanceSpeed = 0.07;

        public static final double driveAngleThreshold =
                12; // angle at which checking angle duration starts, in degrees
        // constant drive up speed
        public static final double driveBalanceSpeed = 0.4;
        // useless for Asheville
        public static final double angularVelocityErrorThreshold = 0.15;
        // coeffiecient of the polynomial function to calculate balancing speed
        public static final double polyCoeff = 1.2;
        // duration of checking for the angle to start autobalance
        public static final double checkDuration = 0.075;
        // override duration for drive up to avoid foul
        public static final double overrideDuration = 4;
        public static final HashMap<String, Command> AutoEventMap = new HashMap<>();

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        public static final double zeroAngleThreshold = 0.15;
        public static final double deadbandValue = 0.195;
        public static final double oscillationTime = 0.06;
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
        public static final double kNoTargetSpeed = 0.3;
    }

    public static final class TapeAlignConstants {
        public static final double kP = .011;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kAngularThreshold = 2;
        public static final double kXOffset = -4;
    }

    public static final class ArmConstants {
        public static final TrapezoidProfile.Constraints kCubeMotionConstraint =
                new TrapezoidProfile.Constraints(0.25, 1.5);
        public static final TrapezoidProfile.Constraints kConeMotionConstraint =
                new TrapezoidProfile.Constraints(0.25, 1.5);
        public static final TrapezoidProfile.Constraints kNotStoringConstraint =
                new TrapezoidProfile.Constraints(0.25, 1.5);
        public static final int kLeftArmMotorCANID = 10;
        public static final int kRightArmMotorCANID = 11;
        public static final double kP = 3.5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        public static final double kArmMOI = 1.57;
        public static final double kMinOutput = -0.9;
        public static final double kMaxOutput = 0.9;
        public static final double kRampRate = 0.125;
        public static final double kConversionFactor = 1;
        public static final int kCurrentLimit = 60;
        public static final double kGearing = 72;
        public static final boolean kEnableForwardLimit = false;
        public static final boolean kEnableReverseLimit = false;
        public static final double kForwardLimit = 0.92;
        public static final double kReverseLimit = 10;
        public static final double kLowWristLimit = 0.2;
        public static final double kHighWristLimit = 0.8;
        public static final double kOuttakeSensorChangeTime = 1.0;
        public static final double kIntakeSensorChangeTime = 3.0;
        public static final double kLowSetpointLimit = 0.16;
        public static final double kHighSetpointLimit = 0.94;
        public static final double kPositionThreshold = 0.025;

        public static final double maxWindPoint = 0.18;
        public static final double YeetpointEnd = 0.77;
        public static final double armWindPoint = 0.412;
    }

    public static final class RollerbarConstants {
        public static final int kRollerbarCANID = 12;
        public static final int kForwardChannel = 11;
        public static final int kReverseChannel = 12;

        public static final double kArmBoundary = 0.23;
        public static final double kDoubleExtensionBoundary = 0.45;
        public static final double kSpinSpeed = -1;

        public static final double kExtendTime = .3;
        public static final double kRetractTime = .5;
    }

    public static final class WristConstants {
        public static final int CANID = 0; //***NEEDS UPDATING***
        
        public static final double kP = 5.8;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kDt = 0.02;
        public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(6, 6);

        public static final double retractedSetpoint = 0.15;
        public static final double extendedSetpoint = -0.15;
        public static final double manualSpeed = 0.01;
        

        public static final double maxAngleRad = 0.2 * 2 * Math.PI;
        public static final double MOI = 0.35;
        public static final double gearing = 50;
        public static final double length = 0.298;
        public static final double minAngleRad = -0.2 * 2 * Math.PI;

    }

    public static final class IntakeConstants {
        public static final double kStartIntakeSpeed = 0.0;
        public static final int kIntakeCANID = 24;
        public static final int kPHCANID = 1;
        public static final double kMaxPressure = 120;
        public static final double kMinPressure = 90;
        public static final int kHiLetGoPort = 0;
        public static final int kForwardChannel = 1;
        public static final int kReverseChannel = 2;
        public static final int kCurrentLimit = 25;

        public static final double kCubeIntakeSpeed = 0.8;
        public static final double kConeIntakeSpeed = 1.0;

        public static final double kCubeSlowIntakeSpeed = 0.5;
        public static final double kConeSlowIntakeSpeed = 0.6;

        public static final double storingCurrentThreshold = 14.7;

        public static final int currentAvgSize = 35;

        public static final double kFlickTime = 0.18;
    }

    public static final class CubeSetpointConstants {
        public static final Setpoints kLowPickup = new Setpoints(0.928, 0.15, -0.5, false);
        public static final Setpoints kStowHigh = new Setpoints(0.5, 0, -0.5, false);
        public static final Setpoints kDoubleFeeder = new Setpoints(0.62, -0.15, -0.5, false);
        public static final Setpoints kLowScore = new Setpoints(0.89, 0.2, -0.3, false);
        public static final Setpoints kMidScore = new Setpoints(0.69, -0.1, -0.5, false);
        public static final Setpoints kHighScore = new Setpoints(0.565, -0.15, -0.75, false);
        public static final Setpoints kStowInFrame = new Setpoints(0.16, 0.15, -0.5, false);
        public static final Setpoints kStowLow = new Setpoints(0.96, 0.2, -0.5, false);
        public static final Setpoints kBackwardsHighScore = new Setpoints(0.432, 0.1, -1.0, false);
        public static final Setpoints kBackwardsMidScore = new Setpoints(0.33, 0, -0.6, false);
        public static final Setpoints kBackwardsLowScore = new Setpoints(0.22, 0.12, -0.5, false);
        public static final Setpoints kBackwardsDoubleFeeder = new Setpoints(0.356, 0.12, -0.5, false);
        public static final Setpoints kBackwardsLowPickup = new Setpoints(0.18, 0.15, 0.0, true);
        public static final Setpoints kBackwardLowPickupAuto = new Setpoints(0.1835, 0.15, 0, true);
    }

    public static final class ConeSetpointConstants {
        public static final Setpoints kLowPickup = new Setpoints(0.94, 0.15, -0.4, false);
        public static final Setpoints kStowHigh = new Setpoints(0.5, 0, -0.4, false);
        public static final Setpoints kDoubleFeeder = new Setpoints(0.62, -0.15, -0.4, false);
        public static final Setpoints kLowScore = new Setpoints(0.87, 0.17, -0.4, false);
        public static final Setpoints kMidScore = new Setpoints(0.72, 0.15, -0.4, false);
        public static final Setpoints kHighScore = new Setpoints(0.64, 0, -0.4, false);
        public static final Setpoints kStowInFrame = new Setpoints(0.16, 0.15, -0.4, false);
        public static final Setpoints kStowLow = new Setpoints(0.96, 0.2, -0.4, false);
    }

    public static class VisionConstants {
        public static final String kSnakeEyesCamera = "OV5647";
        public static final double kCameraHeight = 21;
        public static final double kTargetHeight = 0;
        public static final double kCameraPitchRadians = 0;
        public static final double kTargetAngle = 2.6;

        public static final double nTableDefault = 0.0;

        /** Physical location of the camera on the robot, relative to the center of the robot. */
        public static final Transform3d CAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());

        public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
    }

    public static class LandingGearConstants {
        public static final int kForwardChannel = 8;
        public static final int kBackwardChannel = 9;
    }
}
