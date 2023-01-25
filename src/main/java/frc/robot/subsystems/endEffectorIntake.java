package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import thisporfavor

public class endEffectorIntake extends SubsystemBase {
    
    private final CANSparkMax intakeMotor;
    
    public endEffectorIntake() {
        MotorType brushed;
        intakeMotor = new CANSparkMax(int CANID, brushed);
        //i think that the device id is supposed to be the can
        //im not sure tho
        //it just told me to put a device id so i put that
    }
    
    public void setMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake(double speed) {
        intakeMotor.set(speed = 0);  
    }
}

//ideally next goal would be to maybe add digital input if possible.
//also thinking abt something that would help intake recognize when to stop without using button bindings?
//to prevent something like what happened today with the intake breaking
