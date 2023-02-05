package frc.robot.subsystems;

//motor imports
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//pneumatics imports
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class endEffectorIntake extends SubsystemBase {
   // private DoubleSolenoid solenoid;
    private int change = 0;
    private final PWMSparkMax intakeMotor = new PWMSparkMax(2);

    public endEffectorIntake(){
        //solenoid = new DoubleSolenoid(null, 0, 1);
        SmartDashboard.putNumber("Change", 0);

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
        change += 1;
        intakeMotor.set(0.5);
    }

    public void stopIntake() 
    {
        intakeMotor.set(0);  
    }

    public void slowIntake() 
    {
        intakeMotor.set(-.25);  
    }

    public void outtake()
    {
        change -= 1;
        intakeMotor.set(-0.5);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Change", change);
    }
}