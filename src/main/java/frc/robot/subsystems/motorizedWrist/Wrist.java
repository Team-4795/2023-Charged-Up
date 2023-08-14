package frc.robot.subsystems.motorizedWrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
    WristIO io;
    WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    public static Wrist instance;

    public static Wrist getInstance(){
        if(instance == null){
            switch(Constants.getRobot()){
                case Comp:
                    instance = new Wrist(new WristIOReal()); 
                    break;
                case Sim:
                    instance = new Wrist(new WristIOSim()); 
                    break;
            }
        }
        return instance;
    } 

    public void setExtendedTarget(boolean extended){

    }

    public void flip(){

    }

    public double getEstimatedAngleDeg(){
        return 0;
    }

    public double getSetpointDeg(){
        return 0;
    }


    private Wrist(WristIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(wristInputs);
    }
}
