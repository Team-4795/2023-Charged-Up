package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffectorIntake;

public class IntakeDuration extends CommandBase{
    double duration;
    double speed;
    Timer timer;
    EndEffectorIntake intake;

    public IntakeDuration(EndEffectorIntake intake, double speed, double duration){
        this.intake = intake;
        this.duration = duration;
        this.speed = speed;
        
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        intake.intake(speed);
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(duration);
    }
}