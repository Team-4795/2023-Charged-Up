package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class PowerDistribution {
    PowerDistribution PD = new PowerDistribution(1, ModuleType.kRev);
    // Get the voltage going into the PDP, in increments of 0.05 Volts (can be used for brownout detection in DS)
    double voltage = m_pdp.getVoltage();
    SmartDashboard.putNumber("Voltage", voltage);
    void 
}

