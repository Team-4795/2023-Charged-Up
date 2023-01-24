package frc.robot.commands;

import frc.robot.subsystems.LiftArm;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class LiftCommand extends CommandBase {
    private final LiftArm Lift;
    private double motorSpeed;

    public LiftCommand(LiftArm subsystem, double speed) {
      Lift = subsystem;
      motorSpeed = speed;
      addRequirements(subsystem);
    }

    @Override
    public void initialize() {}


    @Override
    public void execute() {
        Lift.move(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
      return false;
    } 


}