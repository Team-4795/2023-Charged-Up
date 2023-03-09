package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.StateManager;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Vision;

public class AutoScore extends ParallelCommandGroup{
    AutoScore(DriveSubsystem drive, StateManager manager, EndEffectorIntake intake, Vision vision, boolean isCone){
        SequentialCommandGroup sequence;

        addCommands(new TapeAlign(drive, vision, () -> (AutoConstants.VisionXspeed), () -> (AutoConstants.VisionYspeed)),
                    new RunCommand(intake::extend, intake),
                    new SequentialCommandGroup(new RunCommand(manager::pickCube),
                                               new RunCommand(manager::dpadUp),
                                               new WaitCommand(0.5),
                                               new RunCommand(intake::outtake)));
    }
}
