// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutoSelector;
import frc.robot.Constants.AutoConstants;
import frc.robot.StateManager;
import frc.robot.Commands.TapeAlign;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class FreeCubeTripleGamePiece extends SequentialCommandGroup {

  public FreeCubeTripleGamePiece(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
      StateManager m_manager, Vision m_vision, AutoSelector m_autoSelector, Wrist wrist ) {

    PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Free Cube 2 Game Piece 1",
        new PathConstraints(2, 3));
    PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Free Cube 2 Game Piece 2",
        new PathConstraints(2, 3));
    PathPlannerTrajectory CubeThreeGamePiece1 = PathPlanner.loadPath("Free Cube 3 1",
        new PathConstraints(2, 3));   
        PathPlannerTrajectory CubeThreeGamePiece2 = PathPlanner.loadPath("Free Cube 3 2",
        new PathConstraints(2, 3));   

    // Add option of Vision based two game peice split into parts with commands Cube
    addCommands(
        new SequentialCommandGroup(
            drivebase.AutoStartUp(CubeTwoGamePiece1, true, m_intake),
            m_autoSelector.score("cube", "high", m_intake, m_manager, m_arm, drivebase, m_vision, wrist),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(0.01),
            new InstantCommand(wrist::retract, wrist),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)),

            new ParallelCommandGroup(
                drivebase.followTrajectoryCommand(CubeTwoGamePiece1),
                m_autoSelector.intake("cube", m_intake, m_manager, m_arm, wrist)),
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),
            
            new ParallelCommandGroup(  
                drivebase.followTrajectoryCommand(CubeTwoGamePiece2),
                m_autoSelector.score("cube", "mid", m_intake, m_manager, m_arm, drivebase, m_vision, wrist)
               ),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(0.01),
            new InstantCommand(wrist::retract, wrist),
            new InstantCommand(() -> m_intake.setOverrideStoring(false)),

            new ParallelCommandGroup(
                drivebase.followTrajectoryCommand(CubeThreeGamePiece1),
                m_autoSelector.intake("cube", m_intake, m_manager, m_arm, wrist)),
            new InstantCommand(() -> m_intake.setOverrideStoring(true)),  

            new ParallelCommandGroup(  
                drivebase.followTrajectoryCommand(CubeThreeGamePiece2),
                m_autoSelector.score("cube", "low", m_intake, m_manager, m_arm, drivebase, m_vision, wrist)
               ),
            new RunCommand(m_intake::outtake, m_intake).withTimeout(1),
            new InstantCommand(wrist::retract, wrist),
            new InstantCommand(() -> m_intake.setOverrideStoring(false))
          ));
  }
}