package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.StateManager;
//import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.StateManager.Gamepiece;

public class LiftArm extends SubsystemBase {
  private final CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.kLeftArmMotorCANID, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.kRightArmMotorCANID, MotorType.kBrushless);

  private final AbsoluteEncoder liftEncoder;
  private final RelativeEncoder liftRelativeEncoder;
  double requestedSpeed = 0;

  private final SparkMaxPIDController m_PIDController;

  public double setpoint;

  private Timer motionTimer = new Timer();
  private TrapezoidProfile profile;
  private TrapezoidProfile.State targetState;

  // Set when the arm is moving to avoid rollerbar or double extension
  public boolean isTemporary = false;

  public LiftArm(){
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.restoreFactoryDefaults();
    
    liftEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    liftRelativeEncoder = leftArmMotor.getEncoder();

    m_PIDController = leftArmMotor.getPIDController();
    m_PIDController.setFeedbackDevice(liftEncoder);

    // Probably dont need this
    m_PIDController.setPositionPIDWrappingEnabled(false);

    m_PIDController.setP(ArmConstants.kP);
    m_PIDController.setI(ArmConstants.kI);
    m_PIDController.setD(ArmConstants.kD);
    m_PIDController.setFF(ArmConstants.kFF);

    // Temporary values
    m_PIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

    leftArmMotor.setOpenLoopRampRate(ArmConstants.kRampRate);
    rightArmMotor.setOpenLoopRampRate(ArmConstants.kRampRate);
    leftArmMotor.setClosedLoopRampRate(ArmConstants.kRampRate);
    rightArmMotor.setClosedLoopRampRate(ArmConstants.kRampRate);

    liftRelativeEncoder.setPositionConversionFactor(ArmConstants.kConversionFactor);
    liftEncoder.setPositionConversionFactor(ArmConstants.kConversionFactor);

    leftArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
    rightArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);

    // Set relative encoder position to absolute encoder position * 72ticks/rotation
    // TODO: Fix this
    liftRelativeEncoder.setPosition(liftEncoder.getPosition() * ArmConstants.kGearing);

    // TODO: Fix this
    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.kEnableForwardLimit);
    leftArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.kEnableReverseLimit);
    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.kEnableForwardLimit);
    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.kEnableReverseLimit);
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)ArmConstants.kForwardLimit);
    leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)ArmConstants.kReverseLimit);


    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    
    // Set right motor to follow left, inverted
    rightArmMotor.follow(leftArmMotor, true);

    leftArmMotor.setInverted(true);

    leftArmMotor.burnFlash();
    rightArmMotor.burnFlash();

    this.setTargetPosition(this.getPosition());

    motionTimer.start();
    motionTimer.reset();

    this.updateMotionProfile();
  }

  private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(liftEncoder.getPosition(), liftEncoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(setpoint, 0.0);
    switch (StateManager.getGamepiece()) {
      case Cone: profile = new TrapezoidProfile(ArmConstants.kConeMotionConstraint, goal, state); break;
      default: profile = new TrapezoidProfile(ArmConstants.kCubeMotionConstraint, goal, state); break;
    }
    motionTimer.reset();
  }

  // Sets setpoint, where setpoint is 0 to 1
  public void setTargetPosition(double newSetpoint) {
    if (newSetpoint != this.setpoint) {
      this.setpoint = newSetpoint;
      updateMotionProfile();
    }
  }

  public void runAutomatic() {
    double elapsedTime = motionTimer.get();
    if (profile.isFinished(elapsedTime)) {
      targetState = new TrapezoidProfile.State(setpoint, 0.0);
    } else {
      targetState = profile.calculate(elapsedTime);
    }

    m_PIDController.setReference(targetState.position, CANSparkMax.ControlType.kPosition);
  }

  public void runManual() {
    m_PIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }


  // Gets absolute position
  public double getPosition() {
    return liftEncoder.getPosition();
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
    // Logging
    SmartDashboard.putNumber("Relative location", liftRelativeEncoder.getPosition());
    SmartDashboard.putNumber("Absolute location", liftEncoder.getPosition());
    SmartDashboard.putNumber("Applied Speed", rightArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("Desired Speeed", requestedSpeed);
    SmartDashboard.putNumber("Arm setpoint", setpoint);
    SmartDashboard.putBoolean("At arm setpoint", this.atSetpoint());
    SmartDashboard.putNumber("Time to low intake", profile.timeLeftUntil(0.185));
    
    if (targetState != null) {
      SmartDashboard.putNumber("Trapezoidal setpoint", targetState.position);
    }
    liftRelativeEncoder.setPosition(this.getPosition());
  }
}
