package frc.robot.subsystems.intake;

//motor imports
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.StateManager;
//Sensor imports


public class EndEffectorIntake extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeCANID, MotorType.kBrushed);

    public double requestedSpeed = IntakeConstants.kStartIntakeSpeed;

    private static boolean storing = false;
    private boolean currentBasedStoring = false;
    private Timer hasBeenStoring = new Timer();

    private double outtakeSpeed = 0.0;

    private static boolean overrideStoring = false;

    private double[] currentValues = new double[IntakeConstants.currentAvgSize];
    private int oldestIndex = 0;

    public EndEffectorIntake(){
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        intakeMotor.burnFlash();

        hasBeenStoring.start();
        hasBeenStoring.reset();
    }

    public void intakeFromGamepiece(boolean isStowing) {
        double speed = 0;

        if (isStoring()) {
            switch (StateManager.getGamepiece()) {
                case Cube: speed = IntakeConstants.kCubeSlowIntakeSpeed; break;
                case Cone: speed = IntakeConstants.kConeSlowIntakeSpeed; break;
                default: break;
            }
        } else {
            switch (StateManager.getGamepiece()) {
                case Cube: speed = IntakeConstants.kCubeIntakeSpeed; break;
                case Cone: speed = IntakeConstants.kConeIntakeSpeed; break;
                default: break;
            }
        }

        // if (!isStoring() && isStowing) {
        //     speed = 0;
        // }

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

    public void resetStoring() {
        storing = false;
    }

    public static boolean isStoring() {
        // Flip `storing` if overrideStoring is true, otherwise stay the same
        return storing ^ overrideStoring;
    }

    public void setOverrideStoring(boolean override) {
        overrideStoring = override;
    }

    @Override
    public void periodic() {
        if (Robot.isTeleOp()) {
            if(intakeMotor.getOutputCurrent() > 2){
                currentValues[oldestIndex] = intakeMotor.getOutputCurrent();
                oldestIndex++;
                oldestIndex = oldestIndex % currentValues.length;
            }

            if (avgCurrent() > IntakeConstants.storingCurrentThreshold) {
                storing = true;
            } else {
                storing = false;
            }
        }       
        
        SmartDashboard.putNumber("Resistance", intakeMotor.getOutputCurrent() / intakeMotor.getAnalog(Mode.kAbsolute).getVoltage());
        //SmartDashboard.putNumber("Intake Temp", intakeMotor.getMotorTemperature());
        SmartDashboard.putNumber("Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Average Current", avgCurrent());
        SmartDashboard.putNumber("Current Threshold", IntakeConstants.storingCurrentThreshold);
        SmartDashboard.putBoolean("Override Storing", overrideStoring);
        SmartDashboard.putBoolean("Storing?", isStoring());
        SmartDashboard.putBoolean("Current Storing", storing);
        SmartDashboard.putNumber("Requested intake speed", requestedSpeed);
        SmartDashboard.putNumber("Outtake speed", outtakeSpeed);
    }

    public void intake(double speed) {
        requestedSpeed = speed;
         intakeMotor.set(speed);
    }

    private double avgCurrent(){
        double sum = 0;
        for(int i = 0; i < currentValues.length; i++){
            sum += currentValues[i];
        }
        return (sum / currentValues.length);
    }
}