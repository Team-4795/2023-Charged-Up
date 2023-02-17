package frc.robot.subsystems;

//motor imports
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//pneumatics imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.Constants.DriveConstants;

public class EndEffectorIntake extends SubsystemBase {
    private DoubleSolenoid solenoid;
    private Compressor compressor;
    private final PWMSparkMax intakeMotor = new PWMSparkMax(2);
    //public  solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    private final PneumaticHub m_ph = new PneumaticHub(1);
    private final Timer timer = new Timer();
    public EndEffectorIntake(){
    }

    public void extend() {
       solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop() {
        //solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void intake(double speed)
    {
        intakeMotor.set(speed);
    }

    public void intakeTime(double speed, double time)
    {
        timer.start();
        while (timer.get() < 1) {
          SmartDashboard.putNumber("timer", timer.get());
          this.intake(DriveConstants.kOuttakeSpeed);
        }
        timer.stop();
        timer.reset();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pressure", m_ph.getPressure(0));
        SmartDashboard.putNumber("intake speed",intakeMotor.get() );

        
    }
}