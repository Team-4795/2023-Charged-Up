package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.StateManager;
//import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.StateManager.Gamepiece;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.intake.EndEffectorIntake;
import frc.robot.subsystems.wrist.Wrist;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputs inputs = new ArmIOInputs();

  double requestedSpeed = 0;

  public double setpoint;

  private ProfiledPIDController armFeedback = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0, 0));

  Mechanism2d measuredMechanism;
  MechanismRoot2d measuredMechanismRoot;
  MechanismLigament2d measuredArm;

  Mechanism2d setpointMechanism;
  MechanismRoot2d setpointMechanismRoot;
  MechanismLigament2d setpointArm;

  private static final LoggedDashboardNumber armMaxVel = 
    new LoggedDashboardNumber("Arm/MaxVel", 1.5);

  private static final LoggedDashboardNumber armMaxAcc = 
    new LoggedDashboardNumber("Arm/MaxAcc", 1.5);

  private static final LoggedDashboardNumber armkP = 
    new LoggedDashboardNumber("Arm/kP", 1.0);

  public static Pose3d armPose = new Pose3d();

  // Set when the arm is moving to avoid rollerbar or double extension
  public boolean isTemporary = false;

  public Arm(ArmIO io) {
    this.io = io;

    armFeedback.setPID(150, 0, 0);

    measuredMechanism = new Mechanism2d(3, 3);
    measuredMechanismRoot = measuredMechanism.getRoot("Arm", 1.5, 1.5);
    measuredArm = measuredMechanismRoot.append(new MechanismLigament2d("Arm", 1, 0, 6, new Color8Bit(Color.kOrange)));

    setpointMechanism = new Mechanism2d(3, 3);
    setpointMechanismRoot = setpointMechanism.getRoot("Arm", 1.5, 1.5);
    setpointArm = setpointMechanismRoot.append(new MechanismLigament2d("Arm", 1, 0, 6, new Color8Bit(Color.kRed)));

    this.setTargetPosition(this.getPosition());

    this.updateMotionProfile();
  }

  private void updateMotionProfile() {
    // if (!EndEffectorIntake.isStoring()) {
    //   armFeedback.setConstraints(ArmConstants.kNotStoringConstraint);
    // } else if (StateManager.getGamepiece() == StateManager.Gamepiece.Cone) {
    //   armFeedback.setConstraints(ArmConstants.kConeMotionConstraint);
    // } else {
    //   armFeedback.setConstraints(ArmConstants.kCubeMotionConstraint);
    // }
    armFeedback.setConstraints(new TrapezoidProfile.Constraints(armMaxVel.get(), armMaxAcc.get()));
  }

  // Sets setpoint, where setpoint is 0 to 1
  public void setTargetPosition(double newSetpoint) {
    if (newSetpoint != this.setpoint) {
      this.setpoint = newSetpoint;
      updateMotionProfile();
    }
  }

  // Gets absolute position
  public double getPosition() {
    return inputs.armAngleRev;
  }
  
  public boolean atSetpoint() {
    return Math.abs(this.getPosition() - setpoint) < ArmConstants.kPositionThreshold;
  }

  public void resetPosition() {
    this.setTargetPosition(this.getPosition());
    this.updateMotionProfile();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    armFeedback.setConstraints(new TrapezoidProfile.Constraints(
        SmartDashboard.getNumber("Arm/MaxVel", 0.0), SmartDashboard.getNumber("Arm/MaxAcc", 0.0)));

    // Logging
    // SmartDashboard.putNumber("Arm/test vel", SmartDashboard.getNumber("Arm/MaxVel", 0.0));
    // SmartDashboard.putNumber("Arm/test vel2", armMaxVel.get());

    SmartDashboard.putNumber("Relative location", inputs.armAngleRev);
    SmartDashboard.putNumber("Arm/Speed", inputs.armAngleRevPerSec);
    // SmartDashboard.putNumber("Applied Speed", rightArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("Desired Speeed", requestedSpeed);
    SmartDashboard.putNumber("Arm goal", setpoint);
    SmartDashboard.putBoolean("At arm setpoint", this.atSetpoint());
    SmartDashboard.putNumber("Arm applied voltage", inputs.armAppliedVolts);
    SmartDashboard.putNumber("Arm setpoint", armFeedback.getSetpoint().position);
    
    measuredArm.setAngle(inputs.armAngleRev * 360 - 90);
    SmartDashboard.putData("Measured Arm Mechanism", measuredMechanism);

    armPose = new Pose3d(-0.246, 0, 0.767, new Rotation3d(0, -Math.toRadians(inputs.armAngleRev * 360 - 90), 0));
    Logger.getInstance().recordOutput("3D/Arm pose", armPose);

    // Statically access wrist pose, whether its up to date or not
    Logger.getInstance().recordOutput("3D/Wrist pose", new Pose3d(
            Arm.armPose.getTranslation().plus(Wrist.wristPose.getTranslation().rotateBy(Arm.armPose.getRotation())),
            Arm.armPose.getRotation().plus(Wrist.wristPose.getRotation())
        ));

    setpointArm.setAngle(setpoint * 360 - 90);
    SmartDashboard.putData("Setpoint Arm Mechanism", setpointMechanism);

    double feedback = armFeedback.calculate(inputs.armAngleRev, setpoint);
    io.setArmVoltage(feedback);
  }
}
