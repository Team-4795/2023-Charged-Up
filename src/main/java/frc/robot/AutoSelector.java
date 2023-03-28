package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Commands.RollerbarCommand;
import frc.robot.Commands.TapeAlign;
import frc.robot.autoPaths.*;
import frc.robot.subsystems.*;

public class AutoSelector {

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  // Define command which scores gamepeieces at various setpoints
  public Command score(String gamepeice, String setpoint, EndEffectorIntake m_intake, StateManager m_manager,
      LiftArm m_arm, DriveSubsystem drivebase, Vision m_vision, Wrist wrist) {

    if (gamepeice.equals("cube")) {
      if (setpoint.equals("high")) {
        return new SequentialCommandGroup(
          new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            new ParallelCommandGroup(
                new SequentialCommandGroup(   
                    new InstantCommand(() -> m_manager.dpadUp(), m_arm),
                    new ParallelCommandGroup(
                      new RunCommand(m_arm::runAutomatic, m_arm).until(m_arm::atSetpoint),
                      new WaitCommand(0.1)
                        .andThen(new InstantCommand(wrist::extend, wrist))
                        .andThen(new WaitCommand(0.1))
                      )
                    )
                )
        );
      } else if (setpoint.equals("mid")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> m_manager.dpadLeft(), m_arm),
                    new InstantCommand(wrist::retract, wrist),
                    new RunCommand(m_arm::runAutomatic, m_arm).until(m_arm::atSetpoint))
            )
        );
      } else if (setpoint.equals("low")) {
          return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> m_manager.dpadDown(), m_arm),
                    new InstantCommand(wrist::retract, wrist),
                    new RunCommand(m_arm::runAutomatic, m_arm).until(m_arm::atSetpoint)
                )
            )
        );
    }
    } else if (gamepeice.equals("cone")) {

      if (setpoint.equals("mid")) {
        return new SequentialCommandGroup(
          new InstantCommand(m_manager::pickCone),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new InstantCommand(() -> m_manager.dpadLeft(), m_arm),
                  new InstantCommand(wrist::retract, wrist),
                  new RunCommand(m_arm::runAutomatic, m_arm).until(m_arm::atSetpoint)),
             // meanwhile auto align
              new SequentialCommandGroup(
                  new InstantCommand(() -> {
                    m_vision.switchToTape();
                  }),
                  new TapeAlign(
                      drivebase, m_vision,
                      () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))),
          new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
          new InstantCommand(wrist::retract, wrist)
          //not needed with current sensing
          //new InstantCommand(() -> m_intake.setOverrideStoring(false))
          );
      } else if (setpoint.equals("low")) {
        return new SequentialCommandGroup(
          // we don't need this anymore with current sensing
          //new InstantCommand(() -> m_intake.setOverrideStoring(true)),
          new InstantCommand(m_manager::pickCone),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  new InstantCommand(() -> m_manager.dpadDown(), m_arm),
                  new InstantCommand(wrist::retract, wrist),
                  new RunCommand(m_arm::runAutomatic, m_arm).until(m_arm::atSetpoint)),
             // meanwhile auto align
              new SequentialCommandGroup(
                  new InstantCommand(() -> {
                    m_vision.switchToTape();
                  }),
                  new TapeAlign(
                      drivebase, m_vision,
                      () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))),
          new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
          new InstantCommand(wrist::retract, wrist)
          //not needed with current sensing
          //new InstantCommand(() -> m_intake.setOverrideStoring(false))
          );
      }
    }

    return null;

  }

  public Command scoreV2(String gamepiece, String setpoint, EndEffectorIntake m_intake, StateManager m_manager,
  LiftArm m_arm, DriveSubsystem drivebase, Vision m_vision, Wrist wrist){
    InstantCommand positionCommand = new InstantCommand();
    InstantCommand coneOrCube = new InstantCommand();

    switch(gamepiece){
      case "cube":
        coneOrCube = new InstantCommand(m_manager::pickCube);
      case "cone":
        coneOrCube = new InstantCommand(m_manager::pickCone);
    }
    
    switch(setpoint){
      case "high":
        positionCommand = new InstantCommand(() -> m_manager.dpadUp(), m_arm);
      case "mid":
        positionCommand = new InstantCommand(() -> m_manager.dpadLeft(), m_arm);
      case "low":
        positionCommand = new InstantCommand(() -> m_manager.dpadDown(), m_arm);
    }

    return new ParallelCommandGroup(
      new TapeAlign(drivebase, m_vision, () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed).withTimeout(1),
      new SequentialCommandGroup(new InstantCommand(() -> m_intake.setOverrideStoring(true)),
                                 new InstantCommand(wrist::extend),
                                 coneOrCube),
      new SequentialCommandGroup(positionCommand,
                                 new WaitCommand(0.5),
                                 new RunCommand(m_intake::outtake, m_intake).withTimeout(0.5),
                                 new InstantCommand(() -> m_intake.setOverrideStoring(false))
      )
    );
  }

  public Command intake(String gamepeice, EndEffectorIntake m_intake, StateManager m_manager, LiftArm m_arm, Wrist wrist) {
    if (gamepeice.equals("cube")) {
      return new SequentialCommandGroup(
          new InstantCommand(wrist::retract),
          new InstantCommand(m_manager::pickCube),
          new InstantCommand(() -> m_manager.dpadDown()),
          new RunCommand(m_arm::runAutomatic, m_arm).until(m_arm::atSetpoint),
          new RunCommand(() -> m_intake.intakeFromGamepiece(m_manager.isStowing()), m_intake)
              .withTimeout(1),
          new InstantCommand(() -> m_intake.setOverrideStoring(true)));

    } else if (gamepeice.equals("cone")) {
      return new SequentialCommandGroup(
          new InstantCommand(wrist::retract),
          new InstantCommand(m_manager::pickCone),
          new InstantCommand(() -> m_manager.dpadDown()),
          new WaitUntilCommand(m_arm::atSetpoint),
          new RunCommand(() -> m_intake.intakeFromGamepiece(m_manager.isStowing()), m_intake)
              .withTimeout(1),
          new InstantCommand(() -> m_intake.setOverrideStoring(true)));
    }

    return null;
  }

  public Command intakeV2(String gamepiece, EndEffectorIntake m_intake, StateManager m_manager, LiftArm m_arm, Wrist wrist, double duration){
    InstantCommand coneOrCubeCommand = new InstantCommand();
    switch(gamepiece){
      case "cube":
        coneOrCubeCommand = new InstantCommand(m_manager::pickCube);
      case "cone":
        coneOrCubeCommand = new InstantCommand(m_manager::pickCone);
    }

    return new SequentialCommandGroup(
      new InstantCommand(wrist::retract),
      coneOrCubeCommand,
      new InstantCommand(() -> m_manager.dpadDown()),
      new WaitCommand(0.3),
      new RunCommand(() -> m_intake.intakeFromGamepiece(m_manager.isStowing()), m_intake)
          .withTimeout(duration)
    );
  }

  public Command stow(EndEffectorIntake m_intake, StateManager m_manager, Wrist m_wrist, LiftArm m_arm) {
    return 
        new SequentialCommandGroup(
            new InstantCommand(m_manager::pickCube),
            new InstantCommand(() -> m_manager.dpadRight(), m_arm, m_intake),
            new InstantCommand(m_wrist::retract, m_wrist),
            new ParallelDeadlineGroup(
              new RunCommand(m_arm::runAutomatic, m_arm).until(m_arm::atSetpoint),
              new RunCommand(() -> m_intake.intakeFromGamepiece(m_manager.isStowing()), m_intake)));
  }

  public Command outtake(EndEffectorIntake m_intake, StateManager m_manager, Wrist m_wrist, LiftArm m_arm, double time) {
    return new SequentialCommandGroup(
      new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(m_wrist::retract, m_wrist),
                new WaitCommand(IntakeConstants.kFlickTime),
                new RunCommand(m_intake::outtake, m_intake).withTimeout(time)
            ),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(time),
            () -> {
                switch (m_manager.getState()) {
                    case BackwardsHighScore: switch (StateManager.getGamepiece()) {
                        case Cube: return true;
                        default: return false;
                    }
                    default: return false;
                }
            }),
        new InstantCommand(() -> m_intake.setOverrideStoring(false))
    );
  }

  public AutoSelector(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
      StateManager m_manager, Vision m_vision, Wrist wrist, Rollerbar rollerbar) {
    
    chooser.addOption("Free 2 Cube Balance", new Free2CubeBalance(drivebase, m_intake, m_arm, m_field,
      m_manager, m_vision, this, wrist));
    
    chooser.addOption("Cable 2 Cube", new Cable2Cube(drivebase, m_intake, m_arm, m_field,
      m_manager, m_vision, this, wrist));

    chooser.addOption("Center 1 Cube Balance", new Center1CubeBalance(drivebase, m_intake, m_arm, 
      m_manager, m_vision, this, wrist));

    chooser.addOption("Free 2 Cube", new Free2Cube(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    chooser.addOption("Triple Low Cube", new Free3CubeLLL(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    chooser.addOption("Shoooooot", new Shooooot(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    chooser.addOption("Free 2.5 Cube", new Free25Cube(drivebase, m_intake, m_arm, m_field, m_manager, m_vision, this, wrist));

    chooser.addOption("High Cube", new SimpleHighCube(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    chooser.addOption("Mid Cone", new SimpleMidCone(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    chooser.addOption("Cable 2 Balance", new Cable2CubeBalance(drivebase, m_intake, m_arm, m_field,
    m_manager, m_vision, this, wrist));

    chooser.addOption("Triple Cube HML", new Free3CubeHML(drivebase, m_intake, m_arm, m_field, 
    m_manager, m_vision, this, wrist));

    chooser.addOption("Rollerbar Auto", new SequentialCommandGroup(
      new RollerbarCommand(rollerbar, m_arm, m_manager, true),
      new WaitCommand(3),
      new RollerbarCommand(rollerbar, m_arm, m_manager, false)
    ));

    
    

    SmartDashboard.putData("Auto Selector", chooser);

  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}