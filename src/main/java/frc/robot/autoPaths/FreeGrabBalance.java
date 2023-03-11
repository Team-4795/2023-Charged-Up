// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;


import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoSelector;
import frc.robot.StateManager;
import frc.robot.Commands.AutoBalanceOld;
import frc.robot.Commands.DriveCommandOld;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;



public class FreeGrabBalance extends SequentialCommandGroup {

public FreeGrabBalance(DriveSubsystem drivebase, EndEffectorIntake m_intake, LiftArm m_arm, Field2d m_field,
      StateManager m_manager, Vision m_vision, AutoSelector m_autoSelector) {

  PathPlannerTrajectory GrapBalance1 = PathPlanner.loadPath("Balance + grab 1", new PathConstraints(1.5, 1));
  PathPlannerTrajectory GrapBalance2 = PathPlanner.loadPath("Balance + grab 2", new PathConstraints(3, 3));

  addCommands(
    new SequentialCommandGroup(
        drivebase.AutoStartUp(GrapBalance1),
        m_autoSelector.score("cube", "high", m_intake, m_manager, m_arm, drivebase, m_vision),
    
        new ParallelCommandGroup( 
              drivebase.followTrajectoryCommand(GrapBalance1),
    
              new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    m_autoSelector.intake("cube", m_intake, m_manager, m_arm))),
    
        new ParallelCommandGroup(
            drivebase.followTrajectoryCommand(GrapBalance2),
            m_autoSelector.stow(m_intake, m_manager, m_arm)),
    
        new DriveCommandOld(drivebase, -AutoConstants.driveBalanceSpeed, AutoConstants.driveAngleThreshold,AutoConstants.checkDuration).withTimeout(AutoConstants.overrideDuration),
        new AutoBalanceOld(drivebase, AutoConstants.angularVelocityErrorThreshold)));
      }
    }