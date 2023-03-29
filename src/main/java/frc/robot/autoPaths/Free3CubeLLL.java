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
import frc.robot.Commands.TapeAlign;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Rollerbar;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class Free3CubeLLL extends SequentialCommandGroup {

  public Free3CubeLLL(DriveSubsystem drivebase, AutoSelector m_autoSelector) {

    PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Intake Free N2 GP1",
        new PathConstraints(4, 3));
    PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Score Free N1 GP1",
        new PathConstraints(4, 3));
    PathPlannerTrajectory CubeThreeGamePiece1 = PathPlanner.loadPath("Intake Free N1 GP2",
        new PathConstraints(4, 3));   
        PathPlannerTrajectory CubeThreeGamePiece2 = PathPlanner.loadPath("Score Free N3 GP2",
        new PathConstraints(4, 3));   

    // Add option of Vision based two game peice split into parts with commands Cube
    addCommands(
        new SequentialCommandGroup(
            m_autoSelector.autoStartUp(CubeTwoGamePiece1, false),
            m_autoSelector.score("cube", "low", false),
            m_autoSelector.outtake(0.1),

            m_autoSelector.intakeTrajectory("cube", true, CubeTwoGamePiece1),
            m_autoSelector.scoreTrajectory("cube", "low", false, CubeTwoGamePiece2),
            m_autoSelector.outtake(0.1),
            
            m_autoSelector.intakeTrajectory("cube", true, CubeThreeGamePiece1),
            m_autoSelector.scoreTrajectory("cube", "low", false, CubeThreeGamePiece2),
            m_autoSelector.outtake(0.5)
        ));
  }
}