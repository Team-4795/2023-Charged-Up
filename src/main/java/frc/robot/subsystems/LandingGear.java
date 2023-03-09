package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LandingGear extends SubsystemBase{
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
    //placeholders, change later

    public LandingGear(){
    }

    public void land() {
        solenoid.set(Value.kForward);
    }

    public void retract() {
        solenoid.set(Value.kReverse);
    }
}