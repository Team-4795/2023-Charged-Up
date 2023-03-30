// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;
import frc.robot.Constants.AutoConstants;
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.DriveCommandOld;
import frc.robot.subsystems.DriveSubsystem;

public class Cable2CubeBalance extends SequentialCommandGroup {

  public Cable2CubeBalance(DriveSubsystem drivebase, AutoSelector m_autoSelector) {

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
            m_autoSelector.autoStartUp(CubeTwoGamePiece1, false),
            m_autoSelector.score("cube", "high", false),
            m_autoSelector.outtake(0.1),

            m_autoSelector.intakeTrajectory("cube", true, CubeTwoGamePiece1),
            m_autoSelector.scoreTrajectory("cube", "mid", false, CubeTwoGamePiece2),
            m_autoSelector.outtake(0.1),

            m_autoSelector.stowTrajectory(AutoBalance),

            m_autoSelector.autoBalance(true, true)));
  }
}