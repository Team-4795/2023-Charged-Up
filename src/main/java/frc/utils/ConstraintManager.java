package frc.utils;

import org.littletonrobotics.junction.Logger;

import frc.robot.StateManager;
import frc.robot.VirtualSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RollerbarConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.motorizedWrist.Wrist;
import frc.robot.subsystems.rollerbar.Rollerbar;

public class ConstraintManager extends VirtualSubsystem{
    private double armPosition = 0;
    private double armSetpoint = 0;
    private double wristPositon = 0;
    private double wristSetpoint = 0;
    private boolean rollerbarShouldMove = false;
    private TempSetpoint setpoint;
    private boolean temporary = false;
    private boolean needTemp = false;

    private static ConstraintManager instance;

    public static void initialize(){
        if(instance == null){
            instance = new ConstraintManager();
        }
    }

    private ConstraintManager(){
        setpoint = TempSetpoint.None;
    }

    @Override
    public void periodic(){
        this.update(
            Arm.getInstance().getPosition(), 
            Arm.getInstance().getSetpoint(),
            Wrist.getInstance().getPosition(),
            Wrist.getInstance().getGoal(), 
            Rollerbar.getInstance().shouldMove());
        this.applyConstraints();
        Logger.getInstance().recordOutput("Constraints/needTemporary", this.needTemp);
    }

    private void applyConstraints(){
        if(rollerbarShouldMove){
            if(!temporary && (needTemp = needTempSetpoints())){
                setTempSetpoints();
                temporary = true;
            }
        } else if(temporary) {
            StateManager.getInstance().setSetpoints();
            temporary = false;
            needTemp = false;
        }
    }

    private boolean needTempSetpoints(){
        setpoint = TempSetpoint.None;
        if(rollerbarShouldMove){
            if(unsafePosition()){
                setpoint = TempSetpoint.ArmWrist;
                if(!unsafeSetpoint()){
                    return false;
                }
                return true;
            } else if(unsafeSetpoint()){
                if(armPosition > ArmConstants.minSafeRollerbar){
                    return false;
                } else {
                    setpoint = TempSetpoint.ArmOnly;
                    return true;
                }
            }
        }
        return false;
    }

    private void setTempSetpoints(){
        if(setpoint.armPos != -1){
            temporary = true;
            Arm.getInstance().setTargetPosition(setpoint.armPos);
        }
        if(setpoint.wristPos != -1){
            Wrist.getInstance().setTarget(setpoint.wristPos);
        }
    }

    private boolean unsafePosition(){
        if(armPosition < RollerbarConstants.kArmBoundary){
            return true;
        } else if(armPosition < RollerbarConstants.kWristRetractedBoundary && wristPositon < WristConstants.rollerbarSetpoint){
            return true;
        } else {
            return false;
        }
    }

    private boolean unsafeSetpoint(){
        if(armSetpoint < RollerbarConstants.kArmBoundary){
            return true;
        } else if(armSetpoint < RollerbarConstants.kWristRetractedBoundary && wristSetpoint < WristConstants.rollerbarSetpoint){
            return true;
        } else {
            return false;
        }
    }

    private void update(double arm, double armSetpoint, double wrist, double wristSetpoint, boolean rollerbar){
        armPosition = arm;
        this.armSetpoint = armSetpoint;
        wristPositon = wrist;
        this.wristSetpoint = wristSetpoint;
        rollerbarShouldMove = rollerbar;
    } 
}
