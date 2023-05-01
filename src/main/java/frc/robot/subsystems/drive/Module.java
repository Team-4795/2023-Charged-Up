package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.2);
  private final PIDController driveFeedback =
  new PIDController(0.0, 0.0, 0.0, 0.02);
  private final PIDController turnFeedback =
  new PIDController(0.0, 0.0, 0.0, 0.02);

  public SwerveModuleState setpoints = new SwerveModuleState(0.0, new Rotation2d());
  
  private int index;
  
  public Module(ModuleIO io, int index) {
    this.index = index;
    this.io = io;
    
    // switch (index) {
    //   case 0:
    //     chassisAngularOffset = DriveConstants.kFrontLeftChassisAngularOffset;
    //     break;
    //   case 1:
    //     chassisAngularOffset = DriveConstants.kFrontRightChassisAngularOffset;
    //     break;
    //   case 2:
    //     chassisAngularOffset = DriveConstants.kBackLeftChassisAngularOffset;
    //     break;
    //   case 3:
    //     chassisAngularOffset = DriveConstants.kBackRightChassisAngularOffset;
    //     break;
    //   default:
    //     throw new RuntimeException("Invalid module index for Module");
    // }
    
    driveFeedback.setPID(0.2, 0.0, 0.0);

    
    // driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.kDrivingFF, 0.0);
    
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    turnFeedback.setPID(15, 0.0, 0.0);
  }
  
  public void setDesiredState(SwerveModuleState desiredState) { 
    setpoints = desiredState;
   
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, getAngle());

    io.setTurnVoltage(
      turnFeedback.calculate(getAngle().getRadians(), optimizedDesiredState.angle.getRadians())
    );
    
    optimizedDesiredState.speedMetersPerSecond *= Math.cos(turnFeedback.getPositionError());

    double velocityRadPerSec = optimizedDesiredState.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters / 2);

    io.setDriveVoltage(
      driveFeedforward.calculate(velocityRadPerSec)   
        + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec)
    );

    // setpoints = optimizedDesiredState;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityRadPerSec * ModuleConstants.kWheelDiameterMeters / 2,
        new Rotation2d(inputs.turnAbsolutePositionRad));
  }

  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.drivePositionRad / Math.PI / 2 * ModuleConstants.kWheelCircumferenceMeters,
        new Rotation2d(inputs.turnAbsolutePositionRad));
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad / Math.PI / 2 * ModuleConstants.kWheelCircumferenceMeters;
  }
  
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Module" + index, inputs);
  }
}
