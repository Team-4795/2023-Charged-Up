// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ModuleConstants;

import frc.robot.Constants.DriveConstants;


public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double turnAfterEncoderReduction = 150.0 / 7.0;

  public ModuleIOSparkMax(int index) {
    switch (index) {
        case 0:
            driveSparkMax = new CANSparkMax(DriveConstants.kFrontLeftDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless);
            break;
        case 1:
            driveSparkMax = new CANSparkMax(DriveConstants.kFrontRightDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless);
            break;
        case 2:
            driveSparkMax = new CANSparkMax(DriveConstants.kRearLeftDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.kRearLeftTurningCanId, MotorType.kBrushless);
            break;
        case 3:
            driveSparkMax = new CANSparkMax(DriveConstants.kRearRightDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.kRearRightTurningCanId, MotorType.kBrushless);
            break;
        default:
            throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveEncoder = driveSparkMax.getEncoder();
    turnEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    turnEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    driveSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turnSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    // inputs.drivePositionRad =
    //     cleanSparkMaxValue(
    //         inputs.drivePositionRad,
    //         Units.rotationsToRadians(driveEncoder.getPosition()) / driveAfterEncoderReduction);
    // inputs.driveVelocityRadPerSec =
    //     cleanSparkMaxValue(
    //         inputs.driveVelocityRadPerSec,
    //         Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
    //             / driveAfterEncoderReduction);
    // inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();

    // inputs.turnAbsolutePositionRad =
    //     MathUtil.angleModulus(
    //         new Rotation2d(
    //                 turnAbsoluteEncoder.getVoltage()
    //                     / RobotController.getVoltage5V()
    //                     * 2.0
    //                     * Math.PI)
    //             .minus(absoluteEncoderOffset)
    //             .getRadians());
    // inputs.turnPositionRad =
    //     cleanSparkMaxValue(
    //         inputs.turnPositionRad,
    //         Units.rotationsToRadians(turnRelativeEncoder.getPosition())
    //             / turnAfterEncoderReduction);
    // inputs.turnVelocityRadPerSec =
    //     cleanSparkMaxValue(
    //         inputs.turnVelocityRadPerSec,
    //         Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
    //             / turnAfterEncoderReduction);
    // inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
  }

  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}