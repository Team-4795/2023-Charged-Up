package frc.robot.subsystems;

//motor imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


//pneumatics imports
import edu.wpi.first.wpilibj.DoubleSolenoid;

//robot imports
import frc.robot.Constants;

public class endEffectorIntake {
    private DoubleSolenoid solenoid;

    private final CANSparkMax intakeMotor;

    public endEffectorIntake(){
        solenoid = new DoubleSolenoid(null, 0, 1);

        int Intake = Constants.kIntake;
        intakeMotor = new CANSparkMax(Intake, MotorType.kBrushless);
    }

    public void extend() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop() {
        solenoid.set(DoubleSolenoid.Value.kOff);
    }

    //speed
    public void setMotorSpeed()
    {
        intakeMotor.set(0.2);
    }

    public void stopIntake() 
    {
        intakeMotor.set(0);  
    }
}