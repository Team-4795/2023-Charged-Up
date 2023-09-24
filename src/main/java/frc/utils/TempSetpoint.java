package frc.utils;

import frc.robot.Constants.RollerbarConstants;
import frc.robot.Constants.WristConstants;

public enum TempSetpoint {
    None(-1, -1),
    ArmOnly(RollerbarConstants.kWristRetractedBoundary + 0.01, -1),
    ArmWrist(RollerbarConstants.kArmBoundary + 0.01, WristConstants.rollerbarSetpoint);

    double armPos;
    double wristPos;

    private TempSetpoint(double arm, double wrist){
        armPos = arm;
        wristPos = wrist;
    }
}
