package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private SingleJointedArmSim armSim;

    private double armAppliedVolts = 0.0;

    public ArmIOSim() {
        armSim = new SingleJointedArmSim(DCMotor.getNEO(2), 72, 0.3, 0.7, 0, 2 * Math.PI, false);
    }

    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(0.02);

        inputs.armAngleRev = armSim.getAngleRads() / (2 * Math.PI);
        inputs.armAngleRevPerSec = armSim.getVelocityRadPerSec() / (2 * Math.PI);
        inputs.armAppliedVolts = armAppliedVolts;
    }

    public void setArmVoltage(double volts) {
        armAppliedVolts = MathUtil.clamp(volts, -12, 12);
        armSim.setInputVoltage(armAppliedVolts);
    }

}
