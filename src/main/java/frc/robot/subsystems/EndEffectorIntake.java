package frc.robot.subsystems;

//motor imports
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//pneumatics imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.Constants.DriveConstants;

public class EndEffectorIntake extends SubsystemBase {
    private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final PWMSparkMax intakeMotor = new PWMSparkMax(2);
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    private final PneumaticHub m_ph = new PneumaticHub(1);

    public EndEffectorIntake(){
        compressor.enableAnalog(90, 120);
    }

    public void extend() {
       solenoid.set(Value.kForward);
    }

    public void retract() {
        solenoid.set(Value.kReverse);

    }

    public void stop() {
        //solenoid.set(DoubleSolenoid.Value.kOff);

    }

    public void intake(double speed)
    {
        intakeMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pressure", m_ph.getPressure(0));
        SmartDashboard.putBoolean("hamburger", false);


    }
}