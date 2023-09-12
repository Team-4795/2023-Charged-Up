package frc.robot.subsystems.WristV2;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.arm.Arm;

public class Wrist extends SubsystemBase {
    private CANSparkMax wristMotor = new CANSparkMax(WristConstants.CANID, MotorType.kBrushless);
    private AbsoluteEncoder encoder;

    private ArmFeedforward feedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kg, WristConstants.kV,
            WristConstants.ka);
    private ProfiledPIDController controller = new ProfiledPIDController(WristConstants.kP, WristConstants.kI,
            WristConstants.kD, WristConstants.constraints, WristConstants.kDt);
    private double goal;
    private double backupGoal;

    private boolean binaryControl;
    private CircularBuffer wristCurrentBuffer = new CircularBuffer(WristConstants.bufferSize);

    private static Wrist instance;

    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }
        return instance;
    }

    private Wrist() {
        binaryControl = false;
        encoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        goal = this.getPosition();
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
        wristMotor.setSmartCurrentLimit(WristConstants.currentLimit);

        setDefaultCommand(run(() -> {
            double change = MathUtil.applyDeadband(OIConstants.operatorController.getLeftY(), 0.05);
            if (!binaryControl) {
                change = WristConstants.manualSpeed * Math.pow(change, 3);
                goal += change;
                goal = MathUtil.clamp(goal, 0.05, 0.35);
            } else {
                if(change > 0.5){
                    goal = WristConstants.extendedSetpoint;
                } else if(change < -0.5){
                    goal = WristConstants.retractedSetpoint;
                }
            }
        }));
    }

    public void setGoal(double goal) {
        this.goal = goal;
    }

    public void setTarget(double position) {
        goal = position;
    }

    public void retract() {
        goal = WristConstants.retractedSetpoint;
    }

    public void toggleBinaryControl(){
        binaryControl = !binaryControl;
    }

    public void setBinaryControl(boolean on){
        binaryControl = on;
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

    public void extend() {
        goal = WristConstants.extendedSetpoint;
    }

    private double getPosition() {
        return (encoder.getPosition() / 2);
    }

    @Override
    public void periodic() {
        double armPosition = Arm.getInstance().getPosition();
        double position = this.getPosition();
        double wristCurrent = wristMotor.getOutputCurrent();
        double motorSpeed = 0;
        if(binaryControl) {
            wristCurrentBuffer.addLast(wristCurrent);
            if(goal > 0){
                motorSpeed = 0.8;
            } else if (goal < 0){
                motorSpeed = -0.8;
            }
            if(wristCurrent > WristConstants.stallCurrentThreshold){
                motorSpeed *= 0.5;
            }
            wristMotor.set(motorSpeed);
        } else {
            if (armPosition < ArmConstants.kLowWristLimit && goal < 0) {
                backupGoal = WristConstants.extendedSetpoint;
                motorSpeed = feedforward.calculate(backupGoal, 0) + controller.calculate(position, backupGoal);
                wristMotor.set(motorSpeed);
            } else if (armPosition > ArmConstants.kHighWristLimit && goal > 0) {
                backupGoal = WristConstants.retractedSetpoint;
                motorSpeed = feedforward.calculate(backupGoal, 0) + controller.calculate(position, backupGoal);
                wristMotor.set(motorSpeed);
            } else {
                motorSpeed = feedforward.calculate(goal, 0) + controller.calculate(position, goal);
                wristMotor.set(motorSpeed);
            }
        }
        SmartDashboard.putNumber("Wrist Speed", motorSpeed);
        SmartDashboard.putNumber("Wrist Current", wristCurrent);
        SmartDashboard.putBoolean("Wrist Binary Control?", binaryControl);
        SmartDashboard.putNumber("Wrist Goal", goal);
        SmartDashboard.putNumber("Wrist Position", position);
    }

    public double getGoal() {
        return goal;
    }

    public Object setExtendedTarget(boolean b) {
        return null;
    }
}
