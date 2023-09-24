package frc.robot.subsystems.motorizedWrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;

public class WristIOSim implements WristIO{
    
    private SingleJointedArmSim wristSim;
    private double appliedVolts;

    WristIOSim(){
        wristSim = new SingleJointedArmSim(DCMotor.getNEO(1), WristConstants.gearing, WristConstants.MOI, WristConstants.length, WristConstants.minAngleRad, WristConstants.maxAngleRad, false);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        wristSim.update(WristConstants.kDt);

        inputs.angle = wristSim.getAngleRads() / (2 * Math.PI);
        inputs.angularVelocity = wristSim.getVelocityRadPerSec() / (2 * Math.PI);
        inputs.motorVolts = appliedVolts;
        inputs.current = wristSim.getCurrentDrawAmps();
    }


    @Override
    public void set(double motorSpeed) {
        appliedVolts = MathUtil.clamp(motorSpeed, -12, 12);
        wristSim.setInputVoltage(appliedVolts);
    }


    
    
}
