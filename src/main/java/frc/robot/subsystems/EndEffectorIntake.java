package frc.robot.subsystems;

//motor imports
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
//pneumatics imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateManager;
//Sensor imports
import frc.robot.Sensors.HiLetGo;
import frc.utils.Gamepiece;


public class EndEffectorIntake extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeCANID, MotorType.kBrushed);
    private final HiLetGo hiLetGo = new HiLetGo(IntakeConstants.kHiLetGoPort);

    public double requestedSpeed = IntakeConstants.kStartIntakeSpeed;

    private boolean storing = false;
    private Timer hasBeenStoring = new Timer();

    private double outtakeSpeed = 0.0;

    private boolean overrideStoring = false;

    public EndEffectorIntake(){
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        intakeMotor.burnFlash();

        hasBeenStoring.start();
        hasBeenStoring.reset();

    }

    public void intake() {
        double speed = 0;

        if (isStoring()) {
            switch (Gamepiece.getGamepiece()) {
                case Cube: speed = IntakeConstants.kCubeSlowIntakeSpeed; break;
                case Cone: speed = IntakeConstants.kConeSlowIntakeSpeed; break;
                default: break;
            }
        } else {
            switch (Gamepiece.getGamepiece()) {
                case Cube: speed = IntakeConstants.kCubeIntakeSpeed; break;
                case Cone: speed = IntakeConstants.kConeIntakeSpeed; break;
                default: break;
            }
        }

        if (!isStoring() && isStowing) {
            speed = 0;
        }

        requestedSpeed = speed;
        intakeMotor.set(speed);
    }

    public void setOuttakeSpeed(double speed) {
        this.outtakeSpeed = speed;
    }

    public void outtake() {
        requestedSpeed = outtakeSpeed;
        intakeMotor.set(outtakeSpeed);
    }

    private boolean isHiLetGoing(){
        return hiLetGo.isBroken();
    }

    public boolean isStoring() {
        // Flip `storing` if overrideStoring is true, otherwise stay the same
        return storing ^ overrideStoring;
    }

    public void overrideStoring() {
        this.overrideStoring = !this.overrideStoring;
    }

    @Override
    public void periodic() {
        if (storing == isHiLetGoing()) {
            hasBeenStoring.reset();
        }

        double changeTime;
        if (storing) {
            changeTime = ArmConstants.kOuttakeSensorChangeTime;
        } else {
            changeTime = ArmConstants.kIntakeSensorChangeTime;
        }

        if (hasBeenStoring.hasElapsed(changeTime)) {
            storing = !storing;
            hasBeenStoring.reset();
        }

        SmartDashboard.putNumber("Requested intake speed", requestedSpeed);
        SmartDashboard.putNumber("Outtake speed", outtakeSpeed);
        SmartDashboard.putBoolean("HiLetGoing?", isHiLetGoing());
    }

    public void intake(double speed) {
        requestedSpeed = speed;
         intakeMotor.set(speed);
    }
}