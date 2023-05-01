// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;
import frc.robot.subsystems.drive.Drive;



public class SimpleHighCube extends SequentialCommandGroup {

  public SimpleHighCube(Drive drivebase, AutoSelector m_autoSelector) {

    PathPlannerTrajectory AutoBalance = PathPlanner.loadPath("Free Auto Balance", new PathConstraints(3, 3));

    addCommands(
      new SequentialCommandGroup(
        m_autoSelector.autoStartUp(AutoBalance, false),
        m_autoSelector.score("cube", "high", false),
        m_autoSelector.outtake(0.2)
      )
    );
  }
}
