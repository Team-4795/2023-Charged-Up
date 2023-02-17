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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.DriveConstants;

public class EndEffectorIntake extends SubsystemBase {
    private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final CANSparkMax intakeMotor = new CANSparkMax(24, MotorType.kBrushed);
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    private final PneumaticHub m_ph = new PneumaticHub(1);

    public double intakeSpeed = 0.25;
    public double requestedSpeed = 0.25;

    boolean extended = false;

    public EndEffectorIntake(){
        
        intakeMotor.setIdleMode(IdleMode.kBrake);

        intakeMotor.setSmartCurrentLimit(40);
        compressor.enableAnalog(90, 120);
    }

    public void extend() {
        extended = true;
        solenoid.set(Value.kForward);
    }

    public void retract() {
        extended = false;
        solenoid.set(Value.kReverse);
    }

    public void stop() {
        //solenoid.set(DoubleSolenoid.Value.kOff);

    }

    public void setExtended(boolean extend) {
        if (extend) {
            extend();
        } else {
            retract();
        }
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
        SmartDashboard.putBoolean("Wrist extended", extended);

    }
}