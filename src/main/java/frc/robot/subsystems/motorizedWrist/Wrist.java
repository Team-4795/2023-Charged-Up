package frc.robot.subsystems.motorizedWrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.arm.Arm;

public class Wrist extends SubsystemBase{
    private WristIO io;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private ProfiledPIDController wristController = new ProfiledPIDController(
        WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.constraints, WristConstants.kDt
    );
    
    private double goal;

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
                default:
                    instance = new Wrist(new WristIO() {});
                    break;
            }
        }
        return instance;
    } 

    private Wrist(WristIO io){
        this.io = io;
        io.updateInputs(inputs);
        goal = inputs.angle;

        setDefaultCommand(run(() -> {
            double change = MathUtil.applyDeadband(OIConstants.operatorController.getLeftY(), 0.05);
            change = WristConstants.manualSpeed * Math.pow(change, 3);
            goal += change;
            goal = MathUtil.clamp(goal, -0.2, 0.2);
        }));
    }

    public void setTarget(double position){
        goal = position;
    }

    public void retract(){
        goal = WristConstants.retractedSetpoint;
    }

    public void extend(){
        goal = WristConstants.extendedSetpoint;
    }

    public double getAngleDeg(){
        return inputs.angle * 360;
    }

    public double getSetpointDeg(){
        return goal * 360;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        SmartDashboard.putNumber("Wrist goal", goal);
        Logger.getInstance().processInputs("Wrist", inputs);

        double armPosition = Arm.getInstance().getPosition();
        if(armPosition < ArmConstants.kLowWristLimit){
            this.extend();
        } else if(armPosition > ArmConstants.kHighWristLimit){
            this.retract();
        }

        io.set(wristController.calculate(inputs.angle, goal));
    }
}
