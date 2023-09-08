package frc.robot.subsystems.WristV2;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.arm.Arm;

public class Wrist extends SubsystemBase {
    private CANSparkMax wristMotor = new CANSparkMax(WristConstants.CANID, MotorType.kBrushless);
    private AbsoluteEncoder encoder;
    
    private ArmFeedforward feedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kg, WristConstants.kV, WristConstants.ka);
    private ProfiledPIDController controller = new ProfiledPIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.constraints, WristConstants.kDt);
    private double goal;
    private double backupGoal;

    private static Wrist instance;

    public static Wrist getInstance(){
        if(instance == null){
            instance = new Wrist();
        }
        return instance;
    }

    private Wrist(){
        encoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        goal = this.getPosition();

        setDefaultCommand(run(() -> {
            double change = MathUtil.applyDeadband(OIConstants.operatorController.getLeftY(), 0.05);
            change = WristConstants.manualSpeed * Math.pow(change, 3);
            goal += change;
            goal = MathUtil.clamp(goal, -0.2, 0.2);
        }));
    }

    public void setGoal(double goal){
        this.goal = goal;
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

    private double getPosition(){
        return (encoder.getPosition() / 2);
    }

    @Override
    public void periodic(){
        double armPosition = Arm.getInstance().getPosition();
        double position = this.getPosition();
        
        if(armPosition < ArmConstants.kLowWristLimit && goal < 0){
            backupGoal = WristConstants.extendedSetpoint;
            double motorSpeed = feedforward.calculate(backupGoal, 0) + controller.calculate(position, backupGoal);
            wristMotor.set(motorSpeed);
        } else if(armPosition > ArmConstants.kHighWristLimit && goal > 0){
            backupGoal = WristConstants.retractedSetpoint;
            double motorSpeed = feedforward.calculate(backupGoal, 0) + controller.calculate(position, backupGoal);
            wristMotor.set(motorSpeed);
        } else {
            double motorSpeed = feedforward.calculate(goal, 0) + controller.calculate(position, goal);
            wristMotor.set(motorSpeed);
        }
    }
}
