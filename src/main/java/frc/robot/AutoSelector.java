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
import frc.robot.StateManager.State;
import frc.robot.Commands.ChangeStateCommand;
import frc.robot.Commands.RollerbarCommand;
import frc.robot.Commands.TapeAlign;
import frc.robot.autoPaths.*;
import frc.robot.subsystems.*;

public class AutoSelector {
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public Command score(String gamepiece, String setpoint, boolean backwards) {
    switch(gamepiece){
      case "cube": manager.pickCube(); break;
      case "cone": manager.pickCone(); break;
      default: throw new IllegalArgumentException("\"" + gamepiece + "\" is not a valid gamepiece.");
    }

    State state;

    if (backwards) {
      switch (setpoint) {
        case "low": state = State.BackwardsLowPickup; break;
        case "mid": state = State.BackwardsMidScore; break;
        case "high": state = State.BackwardsHighScore; break;
        default: throw new IllegalArgumentException("\"" + setpoint + "\" is not a valid setpoint.");
      }
    } else {
      switch (setpoint) {
        case "low": state = State.LowPickup; break;
        case "mid": state = State.MidScore; break;
        case "high": state = State.HighScore; break;
        default: throw new IllegalArgumentException("\"" + setpoint + "\" is not a valid setpoint.");
      }
    }

    intake.setOverrideStoring(true);

    return new ChangeStateCommand(state, intake, true, arm, wrist, rollerbar, manager);
  }

  public Command scoreTrajectory(String gamepiece, String setpoint, boolean backwards, PathPlannerTrajectory traj) {
    return score(gamepiece, setpoint, backwards).alongWith(drivebase.followTrajectoryCommand(traj));
  }

  public Command intakeTrajectory(String gamepiece, boolean backwards, PathPlannerTrajectory traj) {
    switch(gamepiece){
      case "cube": manager.pickCube(); break;
      case "cone": manager.pickCone(); break;
      default: throw new IllegalArgumentException("\"" + gamepiece + "\" is not a valid gamepiece.");
    }

    StateManager.State state;

    if (backwards) {
      state = State.BackwardsLowPickup;
    } else {
      state = State.LowPickup;
    }

    return new ParallelDeadlineGroup(
      drivebase.followTrajectoryCommand(traj),
      new ChangeStateCommand(state, intake, false, arm, wrist, rollerbar, manager)
    ).andThen(new InstantCommand(() -> intake.setOverrideStoring(true)));
  }

  public Command stow() {
    return new ChangeStateCommand(State.StowInFrame, intake, true, arm, wrist, rollerbar, manager);
  }

  public Command stowTrajectory(PathPlannerTrajectory traj) {
    return stow().alongWith(drivebase.followTrajectoryCommand(traj));
  }

  public Command outtake(double time) {
    return new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(wrist::retract, wrist),
        new WaitCommand(IntakeConstants.kFlickTime),
        new RunCommand(intake::outtake, intake).withTimeout(time)
      ),
      new RunCommand(intake::outtake, intake).withTimeout(time),
      () -> {
        switch (manager.getState()) {
          case BackwardsHighScore: switch (StateManager.getGamepiece()) {
            case Cube: return true;
            default: return false;
          }
          default: return false;
        }
      }).andThen(new InstantCommand(() -> intake.setOverrideStoring(false))
    );
  }

  public Command autoStartUp(PathPlannerTrajectory traj, boolean flip) {
    return drivebase.AutoStartUp(traj, flip, intake);
  }

  DriveSubsystem drivebase;
  EndEffectorIntake intake;
  LiftArm arm;
  StateManager manager;
  Wrist wrist;
  Vision vision;
  Rollerbar rollerbar;

  public AutoSelector(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d field,
      StateManager m_manager, Vision m_vision, Wrist wrist, Rollerbar rollerbar) {

    this.drivebase = drivebase;
    this.intake = m_intake;
    this.arm = m_arm;
    this.manager = m_manager;
    this.vision = m_vision;
    this.wrist = wrist;
    this.rollerbar = rollerbar;


    chooser.addOption("Free 2 Cube Balance", new Free2Cube(drivebase, this));
      
    chooser.addOption("Cable 2 Cube Balance", new Cable2CubeBalance(drivebase, this));
    
    chooser.addOption("Cable 2 Cube", new Cable2Cube(drivebase, this));

    chooser.addOption("Center Cube Balance", new Center1CubeBalance(drivebase, this));

    chooser.addOption("Free 2 Cube Balance", new Free2CubeBalance(drivebase, this));

    chooser.addOption("Free 2 Cube", new Free2Cube(drivebase, this));

    // chooser.addOption("Free Grab Balance", new FreeGrabBalance(drivebase, intake, arm, field,
    //     manager, vision, this, wrist));

    chooser.addOption("Free 2.5 Cube", new Free25Cube(drivebase, this));

    chooser.addOption("Triple Low Cube", new Free3CubeLLL(drivebase, this));

    // chooser.addOption("Shoooooot", new Shooooot(drivebase, intake, arm, field,
    //     manager, vision, this, wrist));

    chooser.addOption("Cable 2 Balance", new Cable2CubeBalance(drivebase, this));

    chooser.addOption("High Cube", new SimpleHighCube(drivebase, this));

    chooser.addOption("Mid Cone", new SimpleMidCone(drivebase, this));

    chooser.addOption("Triple Cube HML", new Free3CubeHML(drivebase, this));

    SmartDashboard.putData("Auto Selector", chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}