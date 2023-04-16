package frc.robot;

import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateManager.Gamepiece;
import frc.robot.StateManager.State;
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.ChangeStateCommand;
import frc.robot.Commands.DriveCommandOld;
import frc.robot.Commands.Yeeeeet;
import frc.robot.autoPaths.*;
import frc.robot.subsystems.*;

public class AutoSelector {
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public Command score(String gamepiece, String setpoint, boolean backwards) {
    Command setGamepiece;

    switch (gamepiece) {
      case "cube":
        setGamepiece = new InstantCommand(manager::pickCube); break;
      case "cone":
        setGamepiece = new InstantCommand(manager::pickCone); break;
      default:
        throw new IllegalArgumentException("\"" + gamepiece + "\" is not a valid gamepiece.");
    }

    State state;

    if (backwards) {
      switch (setpoint) {
        case "low":
          state = State.BackwardsLowPickup;
          break;
        case "mid":
          state = State.BackwardsMidScore;
          break;
        case "high":
          state = State.BackwardsHighScore;
          break;
        default:
          throw new IllegalArgumentException("\"" + setpoint + "\" is not a valid setpoint.");
      }
    } else {
      switch (setpoint) {
        case "low":
          state = State.LowPickup;
          break;
        case "mid":
          state = State.MidScore;
          break;
        case "high":
          state = State.HighScore;
          break;
        default:
          throw new IllegalArgumentException("\"" + setpoint + "\" is not a valid setpoint.");
      }
    }

    Command setOverride = new InstantCommand(() -> intake.setOverrideStoring(true));

    return setGamepiece
      .andThen(setOverride)
      .andThen(new ChangeStateCommand(state, intake, true, arm, wrist, rollerbar, manager));
  }

  public Command scoreTrajectory(String gamepiece, String setpoint, boolean backwards, PathPlannerTrajectory traj) {
    return drivebase.followTrajectoryCommand(traj).alongWith(
      new SequentialCommandGroup(
        new WaitCommand(AutoConstants.kIntakeDelay),
        score(gamepiece, setpoint, backwards)
      )
    );
  }

  public Command intakeTrajectory(String gamepiece, boolean backwards, PathPlannerTrajectory traj) {
    return intakeTrajectory(gamepiece, backwards, traj, 0.0);
  }

  public Command intakeTrajectory(String gamepiece, boolean backwards, PathPlannerTrajectory traj, double time) {
    Command setGamepiece;

    switch (gamepiece) {
      case "cube":
        setGamepiece = new InstantCommand(manager::pickCube); break;
      case "cone":
        setGamepiece = new InstantCommand(manager::pickCone); break;
      default:
        throw new IllegalArgumentException("\"" + gamepiece + "\" is not a valid gamepiece.");
    }

    StateManager.State state;

    if (backwards) {
      state = State.BackwardsLowPickupAuto;
    } else {
      state = State.LowPickup;
    }

    return setGamepiece
      .andThen(
        new ParallelDeadlineGroup(
          drivebase.followTrajectoryCommand(traj).andThen(new WaitCommand(AutoConstants.kIntakeWaitTime)),
          new SequentialCommandGroup(
            new RunCommand(intake::outtake, intake).withTimeout(AutoConstants.kOuttakeDelay), // Run outtake while moving backwards
            new WaitCommand(time),
            new ChangeStateCommand(state, intake, false, arm, wrist, rollerbar, manager)
          )
        ).andThen(new InstantCommand(() -> intake.setOverrideStoring(true)))
      );
  }

  public Command stow() {
    return new ChangeStateCommand(State.StowInFrame, intake, true, arm, wrist, rollerbar, manager);
  }

  public Command stowTrajectory(PathPlannerTrajectory traj) {
    return stow().alongWith(drivebase.followTrajectoryCommand(traj));
  }

  public Command outtake(double time) {
    return new SelectCommand(Map.ofEntries(
      Map.entry(1, new RunCommand(intake::outtake, intake).withTimeout(time)),
      Map.entry(2, new SequentialCommandGroup(
        new InstantCommand(wrist::extend, wrist),
        new WaitCommand(0.3),
        new RunCommand(intake::outtake, intake).withTimeout(time))
      ),
      Map.entry(3, new SequentialCommandGroup(
        new InstantCommand(wrist::retract, wrist),
        new WaitCommand(IntakeConstants.kFlickTime),
        new RunCommand(intake::outtake, intake).withTimeout(time))
      )
    ), () -> {
      if (manager.getState() == State.MidScore && StateManager.getGamepiece() == Gamepiece.Cone) {
        return 2;
      } else if (manager.getState() == State.BackwardsHighScore && StateManager.getGamepiece() == Gamepiece.Cube) {
        return 3;
      } else {
        return 1;
      }
    }).andThen(new InstantCommand(() -> intake.setOverrideStoring(false)));
  }

  public Command autoStartUp(PathPlannerTrajectory traj, boolean flip) {
    return new InstantCommand(intake::resetStoring).andThen(drivebase.AutoStartUp(traj, flip, intake));
  }

  public Command autoBalance(boolean backward, boolean withDriveup) {
    int direction = 1;
    if (backward) {
      direction = -1;
    }
    if (withDriveup) {
      return new SequentialCommandGroup(
          new DriveCommandOld(drivebase, direction * AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold,
              AutoConstants.checkDuration),
          new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold));
    } else {
      return new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold);
    }
  }

  public Command yeeeeet(String gamepiece) {
    return new Yeeeeet(arm, wrist, intake, manager, gamepiece);
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

    chooser.addOption("Free 3 Hybrid MHM", new Free3HybridMHM(this));

    chooser.setDefaultOption("Center 2 + Balance", new Center2CubeBalance(drivebase, this, intake));

    chooser.addOption("Center 1 + Balance", new Center1CubeBalance(this));

    chooser.addOption("Center 1 + Mobility + Balance", new Center1CubeBalanceMobility(this));

    chooser.addOption("Cable 2 Cube", new Cable2Cube(this));

    chooser.addOption("Cable 25", new Cable25HybridMHL(this, intake));

    SmartDashboard.putData("Auto Selector", chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}