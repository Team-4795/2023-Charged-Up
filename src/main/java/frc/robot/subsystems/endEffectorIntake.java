package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class endEffectorIntake extends SubsystemBase {
    
    private final CANSparkMax intakeMotor;
    private final DigitalInput cargoOne, cargoTwo, cargoThree;
    

    public endEffectorIntake() {
        int Intake = Constants.kIntake;
        intakeMotor = new CANSparkMax(Intake, MotorType.kBrushed);
        //i think that the device id is supposed to be the can
        //im not sure tho
        //it just told me to put a device id so i put that

    public void setMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake(double speed) {
        intakeMotor.set(speed = 0);  
    }


}
