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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutoSelector;
import frc.robot.StateManager;
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.DriveCommandOld;
import frc.robot.Commands.TapeAlign;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Rollerbar;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class Shooooot extends SequentialCommandGroup {

    public Shooooot(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
            StateManager m_manager, Vision m_vision, AutoSelector m_autoSelector, Wrist wrist, Rollerbar m_rollerbar) {

        PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Intake Free N2 GP1",
                new PathConstraints(1, 2));
        PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Score Free N2 GP1",
                new PathConstraints(1, 2));
        PathPlannerTrajectory AutoBalance = PathPlanner.loadPath("Free Auto Balance", new PathConstraints(3, 3));
        // Add option of Vision based two game peice split into parts with commands Cube
        addCommands(
                new SequentialCommandGroup(
                        m_autoSelector.autoStartUp(CubeTwoGamePiece1, false),
                        m_autoSelector.scoreV2("cube", "high", false),

                        m_autoSelector.intakeTrajectory("cube", true, CubeTwoGamePiece1),

                        new ParallelCommandGroup(
                                drivebase.followTrajectoryCommand(CubeTwoGamePiece2),
                                m_autoSelector.scoreV2("cube", "high", false)
                        ),

                        new ParallelCommandGroup(
                                drivebase.followTrajectoryCommand(AutoBalance),
                                m_autoSelector.stow()),

                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new DriveCommandOld(drivebase, -AutoConstants.driveBalanceSpeed,
                                                AutoConstants.driveAngleThreshold,
                                                AutoConstants.checkDuration)
                                                .withTimeout(AutoConstants.overrideDuration),
                                        new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold),
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