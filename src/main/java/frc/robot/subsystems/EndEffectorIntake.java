package frc.robot.subsystems;

//motor imports
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//pneumatics imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;

//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

//Sensor imports
import frc.robot.Sensors.HiLetGo;


public class EndEffectorIntake extends SubsystemBase {
    private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeCANID, MotorType.kBrushed);
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.kForwardChannel, IntakeConstants.kReverseChannel);
    private final PneumaticHub m_ph = new PneumaticHub(IntakeConstants.kPHCANID);
    private final HiLetGo hiLetGo = new HiLetGo(IntakeConstants.kHiLetGoPort);

    public double intakeSpeed = IntakeConstants.kOtherIntakeSpeed;
    public double requestedSpeed = IntakeConstants.kRequestedSpeed;

    public boolean extendedTarget = false;
    public boolean extended = false;

    public EndEffectorIntake(){
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        intakeMotor.burnFlash();
        compressor.enableAnalog(IntakeConstants.kMinPressure, IntakeConstants.kMaxPressure);
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

    public void setExtendedTarget(boolean extend) {
        this.extendedTarget = extend;
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

    public boolean isHiLetGoing(){
        return hiLetGo.isBroken();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Pressure", m_ph.getPressure(0));
        SmartDashboard.putNumber("Requested intake speed", requestedSpeed);
        SmartDashboard.putBoolean("Wrist extended target", extendedTarget);
        SmartDashboard.putBoolean("Wrist extended", extended);
        SmartDashboard.putBoolean("HiLetGoing?", isHiLetGoing());
    }
}