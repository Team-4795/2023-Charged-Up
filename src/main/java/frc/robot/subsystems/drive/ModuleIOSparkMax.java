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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class ModuleIOSparkMax implements ModuleIO {
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final double chassisAngularOffset;

    public ModuleIOSparkMax(int index) {
        switch (index) {
            case 0:
                driveSparkMax = new CANSparkMax(DriveConstants.kFrontLeftDrivingCanId, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless);
                chassisAngularOffset = DriveConstants.kFrontLeftChassisAngularOffset;
                break;
            case 1:
                driveSparkMax = new CANSparkMax(DriveConstants.kFrontRightDrivingCanId, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless);
                chassisAngularOffset = DriveConstants.kFrontRightChassisAngularOffset;
                break;
            case 2:
                driveSparkMax = new CANSparkMax(DriveConstants.kRearLeftDrivingCanId, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(DriveConstants.kRearLeftTurningCanId, MotorType.kBrushless);
                chassisAngularOffset = DriveConstants.kBackLeftChassisAngularOffset;
                break;
            case 3:
                driveSparkMax = new CANSparkMax(DriveConstants.kRearRightDrivingCanId, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(DriveConstants.kRearRightTurningCanId, MotorType.kBrushless);
                chassisAngularOffset = DriveConstants.kBackRightChassisAngularOffset;
                break;
            default:
                throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
        }

        driveSparkMax.restoreFactoryDefaults();
        turnSparkMax.restoreFactoryDefaults();

        driveEncoder = driveSparkMax.getEncoder();
        turnEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

        turnEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        driveSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        turnSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        driveSparkMax.enableVoltageCompensation(12.0);
        turnSparkMax.enableVoltageCompensation(12.0);

        driveEncoder.setPosition(0.0);

        driveSparkMax.burnFlash();
        turnSparkMax.burnFlash();
    }

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();

        inputs.turnPositionRad = turnEncoder.getPosition() - chassisAngularOffset;
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
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
