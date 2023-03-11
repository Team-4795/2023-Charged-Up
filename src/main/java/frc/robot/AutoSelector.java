package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Commands.TapeAlign;
import frc.robot.autoPaths.BalanceCubeTwoGamePiece;
import frc.robot.autoPaths.CableAutoBalance;
import frc.robot.autoPaths.CableCubeTwoGamePiece;
import frc.robot.autoPaths.FreeAutoBalance;
import frc.robot.autoPaths.FreeCubeTwoGamePiece;
import frc.robot.autoPaths.FreeGrapBalance;
import frc.robot.autoPaths.FreeGrapCommunity;
import frc.robot.autoPaths.SimpleHighCube;
import frc.robot.autoPaths.SimpleMidCone;
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

  // Define command which scores gamepeieces at various setpoints
  // only the first command is documented
  public Command score(String gamepeice, String setpoint, EndEffectorIntake m_intake, StateManager m_manager,
      LiftArm m_arm, DriveSubsystem drivebase, Vision m_vision) {

    if (gamepeice.equals("cube")) {

      if (setpoint.equals("high")) {
        return new SequentialCommandGroup(
            // ensure the state machine is aware of what we want ie have a game piece and in
            // cube mode
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            // a command group which should finish when the arm gets to the set point
            new ParallelRaceGroup(
                // move arm and extend intake then wait until at set point
                new SequentialCommandGroup(
                    new InstantCommand(() -> m_manager.dpadUp(), m_arm),
                    new InstantCommand(m_intake::extend, m_intake),
                    new WaitUntilCommand(m_arm::atSetpoint)),
                // meanwhile auto align
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                      m_vision.switchToTag();
                    }),
                    new TapeAlign(
                        drivebase, m_vision,
                        () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))),
            // score and then tell statemachine of current state ie no gamepiece
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(m_intake::retract, m_intake),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
      } else if (setpoint.equals("mid")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> m_manager.dpadLeft(), m_arm),
                    new WaitUntilCommand(m_arm::atSetpoint),
                    new InstantCommand(m_intake::extend, m_intake)),
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                      m_vision.switchToTag();
                    }),
                    new TapeAlign(
                        drivebase, m_vision,
                        () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(m_intake::retract, m_intake),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
      } else if (setpoint.equals("low")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> m_manager.dpadDown(), m_arm),
                    new InstantCommand(m_intake::retract, m_intake),
                    new WaitUntilCommand(m_arm::atSetpoint)),
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                      m_vision.switchToTag();
                    }),
                    new TapeAlign(
                        drivebase, m_vision,
                        () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))),
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
      } else if (setpoint.equals("low")) {
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

  public Command scoreV2(String gamepiece, String setpoint, EndEffectorIntake m_intake, StateManager m_manager,
  LiftArm m_arm, DriveSubsystem drivebase, Vision m_vision){
    InstantCommand positionCommand;
    InstantCommand coneOrCube;

    switch(gamepiece){
      case "cube":
        coneOrCube = new InstantCommand(m_manager::pickCube);
      case "cone":
        coneOrCube = new InstantCommand(m_manager::pickCone);
      default:
        coneOrCube = new InstantCommand();
    }
    
    switch(setpoint){
      case "high":
        positionCommand = new InstantCommand(() -> m_manager.dpadUp(), m_arm);
      case "mid":
        positionCommand = new InstantCommand(() -> m_manager.dpadLeft(), m_arm);
      case "low":
        positionCommand = new InstantCommand(() -> m_manager.dpadDown(), m_arm);
      default:
        positionCommand = new InstantCommand();
    }

    return new ParallelCommandGroup(
      new TapeAlign(drivebase, m_vision, () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed).withTimeout(1),
      new SequentialCommandGroup(new InstantCommand(() -> m_intake.setOverrideStoring(true)),
                                 new InstantCommand(m_intake::extend),
                                 coneOrCube),
      new SequentialCommandGroup(positionCommand,
                                 new WaitCommand(0.5),
                                 new RunCommand(m_intake::outtake, m_intake).withTimeout(0.5)
      )
    );
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

  public Command intakeV2(String gamepiece, EndEffectorIntake m_intake, StateManager m_manager, LiftArm m_arm, double duration){
    InstantCommand coneOrCubeCommand = new InstantCommand();
    switch(gamepiece){
      case "cube":
        coneOrCubeCommand = new InstantCommand(m_manager::pickCube);
      case "cone":
        coneOrCubeCommand = new InstantCommand(m_manager::pickCone);
    }

    return new SequentialCommandGroup(
      new InstantCommand(m_intake::retract),
      coneOrCubeCommand,
      new InstantCommand(() -> m_manager.dpadDown()),
      new WaitCommand(0.3),
      new RunCommand(() -> m_intake.intakeFromGamepiece(m_manager.getGamepiece(), m_manager.isStowing()), m_intake)
          .withTimeout(duration)
    );
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
    
    chooser.addOption("2.5 Cube", new BalanceCubeTwoGamePiece(drivebase, m_intake, m_arm, m_field,
      m_manager, m_vision, this));
      
    chooser.addOption("Cable Auto Balence", new CableAutoBalance(drivebase, m_intake, m_arm, m_field,
      m_manager, m_vision, this));
    
    chooser.addOption("Cable 2 Cube", new CableCubeTwoGamePiece(drivebase, m_intake, m_arm, m_field,
      m_manager, m_vision, this));

    chooser.addOption("Free Auto Balance", new FreeAutoBalance(drivebase, m_intake, m_arm, m_field,
      m_manager, m_vision, this));

    chooser.addOption("Free 2 Cube", new FreeCubeTwoGamePiece(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this));

    chooser.addOption("Free Grap community", new FreeGrapCommunity(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this));

    chooser.addOption("Free Grap Balance", new FreeGrapBalance(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this));

    chooser.setDefaultOption("Cable Auto Balance", new CableAutoBalance(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this));

    chooser.addOption("GrapBalance", new FreeGrapBalance(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this));

    chooser.addOption("HighCube", new SimpleHighCube(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this));

    chooser.addOption("MidCone", new SimpleMidCone(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this));

    SmartDashboard.putData("Auto Selector", chooser);

  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}