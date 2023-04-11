package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateManager;
import frc.robot.Constants.ArmConstants;
import frc.robot.StateManager.State;
import frc.robot.subsystems.EndEffectorIntake;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.Wrist;

public class Yeeeeet extends CommandBase {
    LiftArm arm;
    Wrist wrist;
    EndEffectorIntake intake;
    StateManager manager;

    String gamepiece;
    
    double time = 0;
    boolean yeet;

    public Yeeeeet(LiftArm arm, Wrist wrist, EndEffectorIntake intake, StateManager manager, String gamepiece) {
        this.arm = arm;
        this.wrist = wrist;
        this.intake = intake;
        this.manager = manager;
        this.gamepiece = gamepiece;
        yeet = false;
        addRequirements(arm, wrist, intake);
    }

    @Override
    public void initialize() {
        if (arm.getPosition() > ArmConstants.armWindPoint) {
            this.cancel();
        } else {
            switch (gamepiece) {
                case "cone":
                    manager.pickCone();
                default:
                    manager.pickCube();
            }
            arm.setTargetPosition(ArmConstants.YeetpointEnd);
        }
    }

    @Override
    public void execute() {
        arm.setTargetPosition(ArmConstants.YeetpointEnd);
        arm.runAutomatic();
        if (arm.getPosition() > ArmConstants.armWindPoint && !yeet) {
            wrist.extend();
            time = Timer.getFPGATimestamp();
            intake.setOuttakeSpeed(-0.9);
            yeet = true;
        }
        if((Timer.getFPGATimestamp() - time) > 0.5 && yeet){
            intake.outtake();
        }
    }

    @Override
    public boolean isFinished() {
        return arm.atSetpoint();
    }
}