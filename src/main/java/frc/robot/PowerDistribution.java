package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PowerDistribution {
    public PowerDistribution(int i, ModuleType krev) {
    }
    private static final edu.wpi.first.wpilibj.PowerDistribution m_pdp = null;
    PowerDistribution PD = new PowerDistribution(1, ModuleType.kRev);
    // Get the voltage going into the PDP, in increments of 0.05 Volts (can be used for brownout detection in DS)
    double voltage = m_pdp.getVoltage();
    SmartDashboard.putNumber("Voltage", voltage);


    
    // Retrieves PDP temp, in C.
    double temperatureCelsius = m_pdp.getTemperature();
}

