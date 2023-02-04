package frc.robot.subsystems;

//motor imports
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

//pneumatics imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class endEffectorIntake extends SubsystemBase {
   // private DoubleSolenoid solenoid;
    private final PWMVictorSPX intakeMotor = new PWMVictorSPX(0);

    public endEffectorIntake(){
        //solenoid = new DoubleSolenoid(null, 0, 1);

    }

    //public void extend() {
       // solenoid.set(DoubleSolenoid.Value.kForward);
    //}

    //public void retract() {
        //solenoid.set(DoubleSolenoid.Value.kReverse);
    //}

    //public void stop() {
        //solenoid.set(DoubleSolenoid.Value.kOff);
    //}

    //speed
    public void setIntakeSpeed()
    {
        intakeMotor.set(0.5);
    }

    public void stopIntake() 
    {
        intakeMotor.set(0);  
    }
}