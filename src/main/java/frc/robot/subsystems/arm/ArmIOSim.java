package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmIOSim implements ArmIO {
    private SingleJointedArmSim armSim;

    private double armAppliedVolts = 0.0;

    public ArmIOSim() {
        armSim = new SingleJointedArmSim(DCMotor.getNEO(2), ArmConstants.kGearing, ArmConstants.kArmMOI, 0.7, 0, 2 * Math.PI, false);
    }

    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(Constants.DT);

        inputs.angleRev = armSim.getAngleRads() / (2 * Math.PI);
        inputs.angleRevPerSec = armSim.getVelocityRadPerSec() / (2 * Math.PI);
        inputs.appliedVolts = armAppliedVolts;
    }

    public void setArmVoltage(double volts) {
        armAppliedVolts = MathUtil.clamp(volts, -12, 12);
        armSim.setInputVoltage(armAppliedVolts);
    }
}
