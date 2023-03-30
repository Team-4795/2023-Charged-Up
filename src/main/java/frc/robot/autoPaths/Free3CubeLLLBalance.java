package frc.robot.autoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoSelector;
import frc.robot.subsystems.DriveSubsystem;

public class Free3CubeLLLBalance extends SequentialCommandGroup {
    public Free3CubeLLLBalance(DriveSubsystem drivebase, AutoSelector m_autoSelector) {

        PathPlannerTrajectory CubeTwoGamePiece1 = PathPlanner.loadPath("Intake Free N2 GP1",
            new PathConstraints(4, 3));
        PathPlannerTrajectory CubeTwoGamePiece2 = PathPlanner.loadPath("Score Free N1 GP1",
            new PathConstraints(4, 3));
        PathPlannerTrajectory CubeThreeGamePiece1 = PathPlanner.loadPath("Intake Free N1 GP2",
            new PathConstraints(4, 3));   
        PathPlannerTrajectory CubeThreeGamePiece2 = PathPlanner.loadPath("Score Free N3 GP2",
            new PathConstraints(4, 3));   
        PathPlannerTrajectory balanceCommunity = PathPlanner.loadPath("Balance Free N3", 
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
                m_autoSelector.outtake(0.2),
                m_autoSelector.stowTrajectory(balanceCommunity),
                m_autoSelector.autoBalance(false, false)
            ));
      }
}
