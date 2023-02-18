package frc.robot.subsystems;


//motor imports
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//pneumatics imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;

//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//Sensor imports
import frc.robot.Sensors.HiLetGo;


public class EndEffectorIntake extends SubsystemBase {
    private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final CANSparkMax intakeMotor = new CANSparkMax(24, MotorType.kBrushed);
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    private final PneumaticHub m_ph = new PneumaticHub(1);
    private final HiLetGo hiLetGo = new HiLetGo(0);

    public double intakeSpeed = 0.25;
    public double requestedSpeed = 0.25;

    public boolean extendedTarget = false;
    public boolean extended = false;

    private DataLog intakeLog;
    private DoubleLogEntry intake;

    public EndEffectorIntake(){
        intakeMotor.setInverted(true);
        
        intakeMotor.setIdleMode(IdleMode.kBrake);

        intakeMotor.setSmartCurrentLimit(40);
        compressor.enableAnalog(90, 120);

        intakeLog = DataLogManager.getLog();
        intake = new DoubleLogEntry(intakeLog, "/intakeSpeed");
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

    public void setIntakeSpeedFromGamepiece(frc.robot.StateManager.Gamepiece gamepiece) {
        switch (gamepiece) {
            case Cube: setIntakeSpeed(DriveConstants.kCubeIntakeSpeed); break;
            case Cone: setIntakeSpeed(DriveConstants.kConeIntakeSpeed); break;
            default: break;
        }
    }

    public void outtakeFromGamepiece(frc.robot.StateManager.Gamepiece gamepiece) {
        switch (gamepiece) {
            case Cube: intake(DriveConstants.kCubeOuttakeSpeed); break;
            case Cone: intake(DriveConstants.kConeOuttakeSpeed); break;
            default: break;
        }
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

        intake.append(intakeSpeed);
    }
}