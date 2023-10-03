// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager.Gamepiece;
import frc.robot.Commands.AutoCommands;
import frc.robot.Commands.AutoCommands.Height;

public class SimpleHighCube extends AutoPath {

    public Command load(AutoCommands autoCommands) {

        PathPlannerTrajectory AutoBalance = PathPlanner.loadPath("Free Auto Balance", new PathConstraints(3, 3));

        return Commands.sequence(
                autoCommands.autoStartUp(AutoBalance, false),
                autoCommands.score(Gamepiece.Cube, Height.High, false),
                autoCommands.outtake(0.2));
    }
}
