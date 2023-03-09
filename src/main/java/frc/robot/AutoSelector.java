package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autoPaths.CubeTwoGamePiece;
import frc.robot.autoPaths.GrapBalance;
import frc.robot.autoPaths.GrapCommunity;
import frc.robot.autoPaths.PathAutoBalance;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;
public class AutoSelector {

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  // All Path Planner paths
  // Cube 2 Game piece Auto part 1 (from first score to first intake)
  PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Cube 2 Game Piece 1", new PathConstraints(1, 2));
  // Cube 2 Game piece Auto part 2 (from intake to first second score)
  PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Cube 2 Game Piece 2", new PathConstraints(1, 2));

  // Auto Balence with 1 cone
  PathPlannerTrajectory AutoBalance = PathPlanner.loadPath("Auto Balance", new PathConstraints(3, 3));

  // Path using vision from further back for cone.
  PathPlannerTrajectory EarlyVision = PathPlanner.loadPath("Early Vision", new PathConstraints(1, 1));

  PathPlannerTrajectory GrapBalance1 = PathPlanner.loadPath("Balance + grab 1", new PathConstraints(1.5, 1));
  PathPlannerTrajectory GrapBalance2 = PathPlanner.loadPath("Balance + grab 2", new PathConstraints(3, 3));

  public Command score(String gamepeice, String setpoint, EndEffectorIntake m_intake, StateManager m_manager,
      LiftArm m_arm) {

    if (gamepeice.equals("cube")) {

      if (setpoint.equals("high")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            new InstantCommand(() -> m_manager.dpadUp(), m_arm),
            new WaitUntilCommand(m_arm::atSetpoint),
            new InstantCommand(m_intake::extend, m_intake),
            new WaitCommand(0.5),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(m_intake::retract, m_intake),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
      }
     else  if (setpoint.equals("mid")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            new InstantCommand(() -> m_manager.dpadLeft(), m_arm),
            new WaitUntilCommand(m_arm::atSetpoint),
            new InstantCommand(m_intake::extend, m_intake),
            new WaitCommand(0.5),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(m_intake::retract, m_intake),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
      }
      else if (setpoint.equals("low")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            new InstantCommand(() -> m_manager.dpadDown(), m_arm),
            new WaitUntilCommand(m_arm::atSetpoint),
            new InstantCommand(m_intake::retract, m_intake),
            new WaitCommand(0),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
      }
    } else if (gamepeice.equals("cone")) {

      if (setpoint.equals("mid")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCone),
            new InstantCommand(() -> m_manager.dpadLeft(), m_arm),
            new WaitUntilCommand(m_arm::atSetpoint),
            new InstantCommand(m_intake::extend, m_intake),
            new WaitCommand(0.5),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(m_intake::retract, m_intake),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
      }
      else if (setpoint.equals("low")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCone),
            new InstantCommand(() -> m_manager.dpadDown(), m_arm),
            new WaitUntilCommand(m_arm::atSetpoint),
            new InstantCommand(m_intake::retract, m_intake),
            new WaitCommand(0),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
      }
    }

    return null;

  }

  public Command intake(String gamepeice, EndEffectorIntake m_intake, StateManager m_manager, LiftArm m_arm) {
    if (gamepeice.equals("cube")) {
      return new SequentialCommandGroup(
          new InstantCommand(m_intake::retract),
          new InstantCommand(m_manager::pickCube),
          new InstantCommand(() -> m_manager.dpadDown()),
          new WaitUntilCommand(m_arm::atSetpoint),
          new RunCommand(() -> m_intake.intakeFromGamepiece(m_manager.getGamepiece(), m_manager.isStowing()), m_intake)
              .withTimeout(1),
          new InstantCommand(() -> m_intake.setOverrideStoring(true)));

    } else if (gamepeice.equals("cone")) {
      return new SequentialCommandGroup(
          new InstantCommand(m_intake::retract),
          new InstantCommand(m_manager::pickCone),
          new InstantCommand(() -> m_manager.dpadDown()),
          new WaitUntilCommand(m_arm::atSetpoint),
          new RunCommand(() -> m_intake.intakeFromGamepiece(m_manager.getGamepiece(), m_manager.isStowing()), m_intake)
              .withTimeout(1),
          new InstantCommand(() -> m_intake.setOverrideStoring(true)));
    }

    return null;
  }

  public Command stow(EndEffectorIntake m_intake, StateManager m_manager, LiftArm m_arm) {
    return new SequentialCommandGroup(
        new SequentialCommandGroup(
            new InstantCommand(m_manager::pickCube),
            new InstantCommand(() -> m_manager.dpadRight(), m_arm, m_intake),
            new WaitUntilCommand(m_arm::atSetpoint)));
  }

  public AutoSelector(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
      StateManager m_manager, Vision m_vision) {

    // Add option of Vision based two game peice split into parts with commands Cube
    chooser.addOption("CubeTwoGamePiece", new CubeTwoGamePiece(drivebase, m_intake, m_arm, m_field,
     m_manager, m_vision, this));

    // Srinivas idea
    chooser.addOption("Grap community", new GrapCommunity(drivebase, m_intake, m_arm, m_field,
    m_manager, m_vision, this));

    chooser.setDefaultOption("Auto Balance", new PathAutoBalance(drivebase, m_intake, m_arm, m_field,
     m_manager, m_vision, this));
     
    // Add option of Vision based two game peice split into parts with commands Cube
    chooser.addOption("GrapBalance", new GrapBalance(drivebase, m_intake, m_arm, m_field,
     m_manager, m_vision, this));

    SmartDashboard.putData("Auto Selector", chooser);

  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}