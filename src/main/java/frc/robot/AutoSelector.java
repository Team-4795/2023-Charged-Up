package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.DriveCommandOld;
import frc.robot.Commands.PipelineSwitch;
import frc.robot.Commands.TapeAlign;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;

public class AutoSelector {

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final Timer timer = new Timer();
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
      if (setpoint.equals("mid")) {
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
      if (setpoint.equals("low")) {
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
      if (setpoint.equals("low")) {
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

  // Define Auto Selector
  public AutoSelector(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
      StateManager m_manager, Vision m_vision) {

    // Add option of Vision based two game peice split into parts with commands Cube
    chooser.addOption("CubeTwoGamePiece", new SequentialCommandGroup(
        drivebase.AutoStartUp(CubeTwoGamePiece1),
        this.score("cube", "high", m_intake, m_manager, m_arm),

        new ParallelCommandGroup(
            drivebase.followTrajectoryCommand(CubeTwoGamePiece1),
            new SequentialCommandGroup(
                new WaitCommand(1.5),
                this.intake("cube",m_intake,m_manager,m_arm))),

        new ParallelCommandGroup(
            drivebase.followTrajectoryCommand(CubeTwoGamePiece2),
            new SequentialCommandGroup(
                new InstantCommand(() -> m_intake.setOverrideStoring(true)),
                new InstantCommand(m_manager::pickCube),
                new InstantCommand(() -> m_manager.dpadDown(), m_arm),
                new WaitUntilCommand(m_arm::atSetpoint),
                new InstantCommand(m_intake::extend, m_intake))),
        // Align
        new InstantCommand(() -> {
          m_vision.switchToTag();
        }),
        new TapeAlign(
            drivebase,
            m_vision, () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed).withTimeout(1),

        // Run outake for 1 second to scorenew RunCommand(m_intake::outtake,
        // m_intake).withTimeout(1.0),
        new InstantCommand(m_intake::retract, m_intake),
        new InstantCommand(() -> m_intake.setOverrideStoring(false))));

    // Srinivas idea
    chooser.addOption("Grap community", new SequentialCommandGroup(

        drivebase.AutoStartUp(GrapBalance1),

        this.score("cube", "high", m_intake, m_manager, m_arm),

        new ParallelCommandGroup(
            drivebase.followTrajectoryCommand(GrapBalance1),

            new SequentialCommandGroup(
                new WaitCommand(1.5),
                this.intake("cube",m_intake,m_manager,m_arm))),

        new ParallelCommandGroup(
            drivebase.followTrajectoryCommand(EarlyVision),

            new SequentialCommandGroup(
                new InstantCommand(m_manager::pickCube),
                new InstantCommand(() -> m_manager.dpadRight(), m_arm, m_intake),
                new WaitUntilCommand(m_arm::atSetpoint))),

        new TapeAlign(
            drivebase, m_vision,
            () -> AutoConstants.VisionMoveFastX, () -> AutoConstants.VisionMoveFastY).withTimeout(1.5)

    ));

    chooser.setDefaultOption("Auto Balance", new SequentialCommandGroup(

        drivebase.AutoStartUp(AutoBalance),
        this.score("cube", "high", m_intake, m_manager, m_arm),
        new ParallelCommandGroup(
            drivebase.followTrajectoryCommand(AutoBalance),

            new SequentialCommandGroup(
                new InstantCommand(m_manager::pickCube),
                new InstantCommand(() -> m_manager.dpadRight(), m_arm, m_intake),
                new WaitUntilCommand(m_arm::atSetpoint))),

        new DriveCommandOld(drivebase, -AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold,
            AutoConstants.checkDuration).withTimeout(AutoConstants.overrideDuration),
        new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold)));

    // Add option of Vision based two game peice split into parts with commands Cube
    chooser.addOption("GrapBalance", new SequentialCommandGroup(
        drivebase.AutoStartUp(GrapBalance1),
        this.score("cube", "high", m_intake, m_manager, m_arm),

        new ParallelCommandGroup(
            drivebase.followTrajectoryCommand(GrapBalance1),

            new SequentialCommandGroup(
                new WaitCommand(1.5),
                this.intake("cube",m_intake,m_manager,m_arm))),


        new ParallelCommandGroup(
            drivebase.followTrajectoryCommand(GrapBalance2),
            new SequentialCommandGroup(
                new InstantCommand(m_manager::pickCube),
                new InstantCommand(() -> m_manager.dpadRight(), m_arm, m_intake),
                new WaitUntilCommand(m_arm::atSetpoint))),

        new DriveCommandOld(drivebase, -AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold,
            AutoConstants.checkDuration).withTimeout(AutoConstants.overrideDuration),
        new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold)

    ));

    SmartDashboard.putData("Auto Selector", chooser);

  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}