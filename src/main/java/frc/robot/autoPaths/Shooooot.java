// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPaths;

import java.util.Optional;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;


public class Shooooot extends SequentialCommandGroup {

    public Shooooot(AutoSelector m_autoSelector, EndEffectorIntake m_intake, Vision m_vision, Wrist wrist, LiftArm m_arm) {

        PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Free Cube 2 Game Piece 1",
            new PathConstraints(1, 2));
        PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Free Cube 2 Game Piece 2",
            new PathConstraints(1, 2));
        PathPlannerTrajectory AutoBalance = PathPlanner.loadPath("Auto Balance Left", new PathConstraints(3, 3));
        // Add option of Vision based two game peice split into parts with commands Cube
        addCommands(
            new SequentialCommandGroup(
                m_autoSelector.autoStartUp(CubeTwoGamePiece1, true),
                m_autoSelector.scoreV2("cube", "high", Optional.empty()),

                new ParallelCommandGroup(
                    m_autoSelector.followTrajectory(CubeTwoGamePiece1),
                    m_autoSelector.intake("cube")),

                m_autoSelector.scoreV2("cube", "high", Optional.of(CubeTwoGamePiece2)),

                new ParallelCommandGroup(
                    m_autoSelector.followTrajectory(AutoBalance),
                    m_autoSelector.stow()),

                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        m_autoSelector.autoBalanceOld(),
                        new InstantCommand(m_vision::switchToTag)
                    ),

                    new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> m_arm.setTargetPosition(.325), m_arm),
                            new InstantCommand(wrist::retract, wrist),
                            new RunCommand(m_arm::runAutomatic, m_arm).withTimeout(1.5),
                            new RunCommand(() -> m_intake.intake(-1), m_intake).withTimeout(.3)
                        )
                    )
                )
            )
        );
    }
}