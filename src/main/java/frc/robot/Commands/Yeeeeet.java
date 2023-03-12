package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.StateManager;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CubeSetpointConstants;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Wrist;

public class Yeeeeet extends CommandBase{
    LiftArm arm;
    Wrist wrist;
    EndEffectorIntake intake;
    StateManager manager;

    String gamepiece;

    boolean yeet;
    boolean finish;

    public Yeeeeet(LiftArm arm, Wrist wrist, EndEffectorIntake intake, StateManager manager, String gamepiece){
        this.arm = arm;
        this.wrist = wrist;
        this.intake = intake;
        this.manager = manager;
        this.gamepiece = gamepiece;
        yeet = false;
        finish = false;
        addRequirements(arm, wrist, intake);
    }

    @Override
    public void initialize(){
        if(arm.getPosition() > ArmConstants.armWindPoint){
            finish = true;
        }
        switch(gamepiece){
            case "cone": manager.pickCone();
            default: manager.pickCube();
        }
        arm.setTargetPosition(ArmConstants.YeetpointEnd);
        arm.runAutomatic();
    }

    @Override
    public void execute(){
        if(arm.getPosition() > ArmConstants.armWindPoint && !yeet){
            wrist.extend();
            intake.setOuttakeSpeed(-0.9);
            intake.outtake();
            yeet = true;
        }
        if(arm.atSetpoint()){
            finish = true;
        }
    }

    @Override
    public boolean isFinished(){
        return finish;
    }
}
