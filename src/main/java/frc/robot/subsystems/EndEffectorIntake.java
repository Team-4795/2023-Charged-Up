package frc.robot.subsystems;

//motor imports
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//pneumatics imports
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class EndEffectorIntake extends SubsystemBase {
   // private DoubleSolenoid solenoid;
    private final PWMSparkMax intakeMotor = new PWMSparkMax(2);

    public EndEffectorIntake(){
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

    public void intake()
    {
        intakeMotor.set(DriveConstants.kIntakeSpeed);
    }

    public void stopIntake() 
    {
        intakeMotor.set(0);  
    }

    public void slowIntake() 
    {
        intakeMotor.set(DriveConstants.kSlowIntakeSpeed);  
    }

    public void outtake()
    {
        intakeMotor.set(DriveConstants.kOuttakeSpeed);
    }

    @Override
    public void periodic() {}
}