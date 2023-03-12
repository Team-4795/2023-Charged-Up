// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;


public class CableCubeTwoGamePiece extends SequentialCommandGroup {

  public CableCubeTwoGamePiece(AutoSelector m_autoSelector) {

    PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Cable Cube 2 Game Piece 1",
        new PathConstraints(1, 2));
    PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Cable Cube 2 Game Piece 2",
        new PathConstraints(1, 2));

    // Add option of Vision based two game peice split into parts with commands Cube
    addCommands(
        new SequentialCommandGroup(
            m_autoSelector.autoStartUp(CubeTwoGamePiece1, true),
            m_autoSelector.score("cube", "high"),

            new ParallelCommandGroup(
                m_autoSelector.followTrajectory(CubeTwoGamePiece1),
                m_autoSelector.intake("cube")),

                m_autoSelector.followTrajectory(CubeTwoGamePiece2),
            m_autoSelector.score("cube", "mid")));
  }
}