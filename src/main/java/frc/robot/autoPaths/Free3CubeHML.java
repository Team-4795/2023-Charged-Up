// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;
import frc.robot.subsystems.DriveSubsystem;

public class Free3CubeHML extends SequentialCommandGroup {

  public Free3CubeHML(DriveSubsystem drivebase, AutoSelector m_autoSelector) {

    PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Intake Free N2 GP1",
        new PathConstraints(4, 3));
    PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Score Free N2 GP1",
        new PathConstraints(4, 3));
    PathPlannerTrajectory CubeThreeGamePiece1 = PathPlanner.loadPath("Intake Free N2 GP2",
        new PathConstraints(4, 3));   
        PathPlannerTrajectory CubeThreeGamePiece2 = PathPlanner.loadPath("Score Free N3 GP2",
        new PathConstraints(4, 3));   

    addCommands(
        new SequentialCommandGroup(
            m_autoSelector.autoStartUp(CubeTwoGamePiece1, false),
            m_autoSelector.score("cube", "high", false),
            m_autoSelector.outtake(0.2),

            m_autoSelector.intakeTrajectory("cube", true, CubeTwoGamePiece1),
            m_autoSelector.scoreTrajectory("cube", "mid", false, CubeTwoGamePiece2),
            m_autoSelector.outtake(0.3),
            
            m_autoSelector.intakeTrajectory("cube", true, CubeThreeGamePiece1),
            m_autoSelector.scoreTrajectory("cube", "low", false, CubeThreeGamePiece2),
            m_autoSelector.outtake(0.4)
        ));
  }
}