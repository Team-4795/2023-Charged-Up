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
import frc.robot.subsystems.Rollerbar;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class Cable2Cube extends SequentialCommandGroup {

  public Cable2Cube(DriveSubsystem drivebase, AutoSelector m_autoSelector) {

    PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Intake Cable N2 GP4",
        new PathConstraints(3.5, 3.5));
    PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Score Cable N2 GP4",
        new PathConstraints(3.5, 3.5));

    // Add option of Vision based two game peice split into parts with commands Cube
    addCommands(
        new SequentialCommandGroup(
            m_autoSelector.autoStartUp(CubeTwoGamePiece1, false),
            m_autoSelector.scoreV2("cube", "high", false),
            m_autoSelector.outtake(0.1),

            m_autoSelector.intakeTrajectory("cube", true, CubeTwoGamePiece1),

            new ParallelCommandGroup(
                drivebase.followTrajectoryCommand(CubeTwoGamePiece2),
                m_autoSelector.scoreV2("cube", "mid", false)
            ),

            m_autoSelector.outtake(0.5)
        ));
  }
}