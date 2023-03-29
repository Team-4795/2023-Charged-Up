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
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.DriveCommandOld;
import frc.robot.Commands.TapeAlign;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Rollerbar;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class Cable2CubeBalance extends SequentialCommandGroup {

  public Cable2CubeBalance(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
      StateManager m_manager, Vision m_vision, AutoSelector m_autoSelector, Wrist wrist, Rollerbar m_rollerbar) {

    PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Intake Cable N2 GP4",
        new PathConstraints(3.5, 3.5));
    PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Score Cable N2 GP4",
        new PathConstraints(3.5, 3.5));

    PathPlannerTrajectory AutoBalance = PathPlanner.loadPath("Balance Cable Community", 
         new PathConstraints(3.5, 3.5));

    PathPlannerTrajectory AutoDriveUp = PathPlanner.loadPath("TEST Cable Balance Driveup", 
         new PathConstraints(3.5, 3.5));

    // Add option of Vision based two game peice split into parts with commands Cube
    addCommands(
        new SequentialCommandGroup(
            drivebase.AutoStartUp(CubeTwoGamePiece1,true, m_intake),
            m_autoSelector.score("cube", "high", m_intake, m_manager, m_arm, drivebase, m_vision, wrist),
            m_autoSelector.outtake(m_intake, m_manager, wrist, m_arm, 0.2),

            new ParallelCommandGroup(
                drivebase.followTrajectoryCommand(CubeTwoGamePiece1),
                m_autoSelector.intake("cube", m_intake, m_manager, m_arm, wrist, m_rollerbar)),

            new ParallelCommandGroup(
                drivebase.followTrajectoryCommand(CubeTwoGamePiece2),
                m_autoSelector.score("cube", "mid", m_intake, m_manager, m_arm, drivebase, m_vision, wrist)),

            m_autoSelector.outtake(m_intake, m_manager, wrist, m_arm, 0.2),

            new ParallelCommandGroup(
                drivebase.followTrajectoryCommand(AutoBalance),
                m_autoSelector.stow(m_intake, m_manager, wrist, m_arm)),
            //drivebase.followTrajectoryCommand(AutoDriveUp),

            new DriveCommandOld(drivebase, AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold,
                AutoConstants.checkDuration).withTimeout(AutoConstants.overrideDuration),
            new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold)));
  }
}