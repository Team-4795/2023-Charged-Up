package frc.robot.subsystems;

//motor imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//pneumatics imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.Constants.DriveConstants;

public class EndEffectorIntake extends SubsystemBase {
    private DoubleSolenoid solenoid;
    private Compressor compressor;
    private final CANSparkMax intakeMotor = new CANSparkMax(24, MotorType.kBrushed);
    //public  solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    private final PneumaticHub m_ph = new PneumaticHub(1);

    public double intakeSpeed = 0.25;
    public double requestedSpeed = 0.25;

    public EndEffectorIntake(){
        
        intakeMotor.setIdleMode(IdleMode.kBrake);

        intakeMotor.setSmartCurrentLimit(40);
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

    public void setIntakeSpeed(double speed) {
        this.requestedSpeed = speed;
        this.intakeSpeed = speed;
    }

    public void intakeAutomatic() {
        this.requestedSpeed = intakeSpeed;
        intakeMotor.set(intakeSpeed);
    }

    public void intake(double speed) {
        this.requestedSpeed = speed;
        intakeMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pressure", m_ph.getPressure(0));
        SmartDashboard.putNumber("Requested intake speed", requestedSpeed);
    }
}