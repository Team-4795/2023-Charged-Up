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
import frc.robot.AutoSelector;
import frc.robot.StateManager;
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.DriveCommandOld;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;



public class SimpleHighCube extends SequentialCommandGroup {

  public SimpleHighCube(DriveSubsystem drivebase, AutoSelector m_autoSelector) {

    PathPlannerTrajectory AutoBalance = PathPlanner.loadPath("Free Auto Balance", new PathConstraints(3, 3));

    addCommands(
      new SequentialCommandGroup(
        m_autoSelector.autoStartUp(AutoBalance, false),
        m_autoSelector.scoreV2("cube", "high", false),
        m_autoSelector.outtake(0.2)
      )
    );
  }
}
