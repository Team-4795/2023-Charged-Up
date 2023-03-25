// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerbarConstants;

public class Rollerbar extends SubsystemBase {
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RollerbarConstants.kForwardChannel, RollerbarConstants.kForwardChannel);
  private final CANSparkMax rollerMotor = new CANSparkMax(RollerbarConstants.kRollerbarCANID, MotorType.kBrushless);

  private boolean extended = getExtension();
  public boolean targetExtend = getExtension();

  public Rollerbar() {
    rollerMotor.restoreFactoryDefaults();
    rollerMotor.setIdleMode(IdleMode.kBrake);
    rollerMotor.burnFlash();
  }

  private boolean getExtension() {
    switch (solenoid.get()) {
      case kForward: return true;
      case kReverse: return false;
      case kOff: return false;
      default: return false;
    }
  }

  private void extend() {
    extended = true;
    solenoid.set(Value.kForward);
  }

  private void retract() {
    extended = false;
    solenoid.set(Value.kReverse);
  }

  private void move() {
    if (targetExtend) {
      extend();
    } else {
      retract();
    }
  }

  public void tryExtend() {
    targetExtend = true;
  }

  public void tryRetract() {
    targetExtend = false;
  }

  public void tryMove(double position) {
    if (position > RollerbarConstants.kBoundary) {
      move();
    }
  }

  public boolean isExtended() {
    return extended;
  }

  public void setExtendedTarget(boolean extend) {
    targetExtend = extend;
  }

  public void spin() {
    rollerMotor.set(RollerbarConstants.kSpinSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Target rollerbar extension", targetExtend);
    SmartDashboard.putBoolean("Rollerbar extension", targetExtend);
  }
}
