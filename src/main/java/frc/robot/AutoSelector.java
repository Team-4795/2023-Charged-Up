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
import frc.robot.autoPaths.CenterScoreBalance;
import frc.robot.autoPaths.FreeAutoBalance;
import frc.robot.autoPaths.FreeCubeTwoGamePiece;
import frc.robot.autoPaths.FreeGrabBalance;
import frc.robot.autoPaths.FreeGrabCommunity;
import frc.robot.autoPaths.Shooooot;
import frc.robot.autoPaths.SimpleHighCube;
import frc.robot.autoPaths.SimpleMidCone;
import frc.robot.autoPaths.TwoScoreOnePickup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class AutoSelector {

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  // Define command which scores gamepeieces at various setpoints
  // only the first command is documented
  public Command score(String gamepeice, String setpoint, EndEffectorIntake m_intake, StateManager m_manager,
      LiftArm m_arm, DriveSubsystem drivebase, Vision m_vision, Wrist wrist) {

    if (gamepeice.equals("cube")) {

      if (setpoint.equals("high")) {
        return new SequentialCommandGroup(
            // ensure the state machine is aware of what we want ie have a game piece and in
            // cube mode
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            // a command group which should finish when the arm gets to the set point
            new ParallelCommandGroup(
                // move arm and extend intake then wait until at set point
                new SequentialCommandGroup(
                    new InstantCommand(() -> m_manager.dpadUp(), m_arm),
                    new InstantCommand(wrist::retract, wrist),
                    new RunCommand(m_arm::runAutomatic, m_arm).withTimeout(1.5)
                // meanwhile auto align
                // new SequentialCommandGroup(
                //     new InstantCommand(() -> {
                //       m_vision.switchToTag();
                //     }),
                //     new TapeAlign(
                //         drivebase, m_vision,
                //         () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed))
                        
                        )),
            // score and then tell statemachine of current state ie no gamepiece
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(wrist::retract, wrist),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
      } else if (setpoint.equals("mid")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> m_manager.dpadLeft(), m_arm),
                    new RunCommand(m_arm::runAutomatic, m_arm).withTimeout(1.5),
                    new InstantCommand(wrist::retract, wrist))
                    
                // new SequentialCommandGroup(
                //     new InstantCommand(() -> {
                //       m_vision.switchToTag();
                //     }),
                //     new TapeAlign(
                //         drivebase, m_vision,
                //         () -> AutoConstants.VisionXspeed, () -> AutoConstants.VisionYspeed)
                        ),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(wrist::retract, wrist),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
      } else if (setpoint.equals("low")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCube),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> m_manager.dpadDown(), m_arm),
                    new InstantCommand(wrist::retract, wrist),
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
            new InstantCommand(wrist::extend, wrist),
            new WaitCommand(0.5),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(wrist::retract, wrist),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
      } else if (setpoint.equals("low")) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            new InstantCommand(m_manager::pickCone),
            new InstantCommand(() -> m_manager.dpadDown(), m_arm),
            new WaitUntilCommand(m_arm::atSetpoint),
            new InstantCommand(wrist::retract, wrist),
            new WaitCommand(0),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1.0),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)));
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
          new RunCommand(m_arm::runAutomatic, m_arm).withTimeout(1.5),
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

  public Command stow(EndEffectorIntake m_intake, StateManager m_manager, LiftArm m_arm) {
    return 
        new SequentialCommandGroup(
            new InstantCommand(m_manager::pickCube),
            new InstantCommand(() -> m_manager.dpadRight(), m_arm, m_intake),

            new RunCommand(m_arm::runAutomatic, m_arm).withTimeout(1.5),
            new RunCommand(() -> m_intake.intakeFromGamepiece(m_manager.getGamepiece(), m_manager.isStowing()), m_intake)
            .withTimeout(.5));
  }

  public AutoSelector(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
      StateManager m_manager, Vision m_vision, Wrist wrist) {
    
    chooser.addOption("2.5 Cube", new BalanceCubeTwoGamePiece(drivebase, m_intake, m_arm, m_field,
      m_manager, m_vision, this, wrist));
      
    chooser.addOption("Cable Auto Balance", new CableAutoBalance(drivebase, m_intake, m_arm, m_field,
      m_manager, m_vision, this, wrist));
    
    chooser.addOption("Cable 2 Cube", new CableCubeTwoGamePiece(drivebase, m_intake, m_arm, m_field,
      m_manager, m_vision, this, wrist));

    chooser.addOption("Center Score Balance", new CenterScoreBalance(drivebase, m_intake, m_arm, 
      m_manager, m_vision, this, wrist));

    chooser.addOption("Free Auto Balance", new FreeAutoBalance(drivebase, m_intake, m_arm, m_field,
      m_manager, m_vision, this, wrist));

    chooser.addOption("Free 2 Cube", new FreeCubeTwoGamePiece(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    chooser.addOption("Free Grab Balance", new FreeGrabBalance(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    chooser.addOption("Free Grab community", new FreeGrabCommunity(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    chooser.addOption("Shoooooot", new Shooooot(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    //chooser.addOption("Two Score One Pickup", new TwoScoreOnePickup(drivebase, m_intake, m_arm, m_field,
      //  m_manager, m_vision, this));

    chooser.addOption("High Cube", new SimpleHighCube(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    chooser.addOption("Mid Cone", new SimpleMidCone(drivebase, m_intake, m_arm, m_field,
        m_manager, m_vision, this, wrist));

    

    SmartDashboard.putData("Auto Selector", chooser);

  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}