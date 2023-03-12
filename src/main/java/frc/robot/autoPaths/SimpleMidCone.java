// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;


public class SimpleMidCone extends SequentialCommandGroup {

    public SimpleMidCone(AutoSelector m_autoSelector) {

        PathPlannerTrajectory AutoBalance = PathPlanner.loadPath("Auto Balance Left", new PathConstraints(3, 3));

        addCommands(
            new SequentialCommandGroup(
                m_autoSelector.autoStartUp(AutoBalance, true),
                m_autoSelector.score("cone", "mid")));
    }
}
