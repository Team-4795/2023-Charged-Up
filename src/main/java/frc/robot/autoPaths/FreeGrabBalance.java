// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPaths;

import java.util.Optional;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;


public class FreeGrabBalance extends SequentialCommandGroup {

    public FreeGrabBalance(AutoSelector m_autoSelector) {

        PathPlannerTrajectory GrapBalance1 = PathPlanner.loadPath("Balance + grab 1", new PathConstraints(1.5, 1));
        PathPlannerTrajectory GrapBalance2 = PathPlanner.loadPath("Balance + grab 2", new PathConstraints(3, 3));

        addCommands(
            new SequentialCommandGroup(
                m_autoSelector.autoStartUp(GrapBalance1, true),
                m_autoSelector.scoreV2("cube", "high", Optional.empty()),

                new ParallelCommandGroup(
                    m_autoSelector.followTrajectory(GrapBalance1),
                    m_autoSelector.intakeV2("cube", 0.7)),

                new ParallelCommandGroup(
                    m_autoSelector.followTrajectory(GrapBalance2),
                    m_autoSelector.stow()),

                m_autoSelector.autoBalanceOld()));
    }
}