// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerbarConstants;

public class Rollerbar extends SubsystemBase {
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RollerbarConstants.kForwardChannel, RollerbarConstants.kReverseChannel);
  private final CANSparkMax rollerMotor = new CANSparkMax(RollerbarConstants.kRollerbarCANID, MotorType.kBrushed);

  private boolean movingToExtended = getExtension();
  private boolean targetExtend = getExtension();
  private boolean extended = getExtension();

  private Timer extensionTimer = new Timer();

  public Rollerbar() {
    rollerMotor.restoreFactoryDefaults();
    rollerMotor.setIdleMode(IdleMode.kBrake);
    rollerMotor.setSmartCurrentLimit(30);
    rollerMotor.burnFlash();

    extensionTimer.reset();
    extensionTimer.start();
  }  

  public boolean getTarget(){
    return targetExtend;
  }

  public void setTarget(boolean target){
    targetExtend = target;
  }

  public void tryMove(double position) {
    if (position > RollerbarConstants.kArmBoundary) {
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

  public void stop(){
    rollerMotor.set(0);
  }

  @Override
  public void periodic() {
    // Set `extended` on a delay to account for physical movement time
    if (extensionTimer.hasElapsed(1)) {
      extended = movingToExtended;
    }

    SmartDashboard.putBoolean("Target rollerbar extension", targetExtend);
    SmartDashboard.putBoolean("Rollerbar extension", extended);
    SmartDashboard.putBoolean("movingToExtended", movingToExtended);
    SmartDashboard.putBoolean("solenoid extended", getExtension());
  }

  private boolean getExtension() {
    switch (solenoid.get()) {
      case kForward: return true;
      default: return false;
    }
  }

  public void extend() {
    movingToExtended = true;
    solenoid.set(Value.kForward);
  }

  public void retract() {
    movingToExtended = false;
    solenoid.set(Value.kReverse);
  }

  private void move() {
    if (targetExtend) {
      extend();
    } else {
      retract();
    }
  }
}
