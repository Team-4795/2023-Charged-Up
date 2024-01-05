package frc.robot.subsystems.motorizedWrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.CircularBuffer;
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

    private boolean binaryControl;
    private CircularBuffer wristCurrentBuffer = new CircularBuffer(WristConstants.bufferSize);

    private ArmFeedforward feedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kg, WristConstants.kV,
            WristConstants.ka);
    private ProfiledPIDController controller = new ProfiledPIDController(
        WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.constraints, WristConstants.kDt
    );
    
    private double goal;
    private double backupGoal;

    private static Wrist instance;

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
        binaryControl = false;
        io.updateInputs(inputs);
        goal = inputs.angle;

        setDefaultCommand(run(() -> {
            double change = MathUtil.applyDeadband(OIConstants.operatorController.getLeftY(), 0.05);
            change = WristConstants.manualSpeed * Math.pow(change, 3);
            goal += change;
            goal = MathUtil.clamp(goal, WristConstants.minPositionRev, WristConstants.maxPositionRev);
        }));
    }

    public void setTarget(double position){
        goal = position;
    }

    public void zero(){
        io.zero();
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

    public double getGoal(){
        return goal;
    }

    public double getPosition(){
        return inputs.angle;
    }

    public void flip() {
        double dist1 = Math.abs(goal - WristConstants.retractedSetpoint);
        double dist2 = Math.abs(goal - WristConstants.extendedSetpoint);
        if (dist1 < dist2) {
            goal = WristConstants.extendedSetpoint;
        } else {
            goal = WristConstants.retractedSetpoint;
        }
    }

    public void setExtendedTarget(boolean b){}

    public void toggleBinaryControl(){
        binaryControl = !binaryControl;
    }

    public void setBinaryControl(boolean on){
        binaryControl = on;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().recordOutput("Wrist/Goal", goal);
        Logger.getInstance().processInputs("Wrist", inputs);
        double motorSpeed = 0;
        double armPosition = Arm.getInstance().getPosition();
        double position = this.getPosition();
        boolean backup = false;
        if(binaryControl){
            backup = armPosition > ArmConstants.kHighWristLimit && goal > 0;
            wristCurrentBuffer.addLast(inputs.current);
            if(backup){
                motorSpeed = -0.6;
            } else {
                if(goal > WristConstants.midPoint){
                    motorSpeed = 0.6;
                } else {
                    motorSpeed = -0.6;
                }
            }
            if(this.avgCurrent() > WristConstants.stallCurrentThreshold){
                motorSpeed *= 0.4;
            }
            io.set(motorSpeed);
        } else {
            if (armPosition < ArmConstants.kLowWristLimit && goal < WristConstants.midPoint) {
                backupGoal = WristConstants.extendedSetpoint;
                motorSpeed = feedforward.calculate(FFCalc(backupGoal), 0) + controller.calculate(position, backupGoal);
                io.set(motorSpeed);
            } else if (armPosition > ArmConstants.kHighWristLimit && goal > WristConstants.midPoint) {
                backupGoal = WristConstants.retractedSetpoint;
                motorSpeed = feedforward.calculate(FFCalc(backupGoal), 0) + controller.calculate(position, backupGoal);
                io.set(motorSpeed);
            } else {
                motorSpeed = feedforward.calculate(FFCalc(goal), 0) + controller.calculate(position, goal);
                io.set(motorSpeed);
            }
        }
        
        Logger.getInstance().recordOutput("Wrist/motorSpeed", motorSpeed);
        Logger.getInstance().recordOutput("Wrist/AvgCurrent", avgCurrent());
    }

    private double FFCalc(double goal){
        double offset = goal + (WristConstants.maxPositionRev - WristConstants.minPositionRev) / 2;
        offset *= (2 * Math.PI);
        return offset;
    }

    private double avgCurrent() {
        double sum = 0;
        for (int i = 0; i < wristCurrentBuffer.size(); i++) {
            sum += wristCurrentBuffer.get(i);
        }
        return sum / wristCurrentBuffer.size();
    }
}
