package frc.robot.subsystems;

//motor imports
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//pneumatics imports
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//imp/ort edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.PneumaticHub;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

//robot imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.Constants.DriveConstants;

public class EndEffectorIntake extends SubsystemBase {
    private final PWMSparkMax intakeMotor = new PWMSparkMax(2);

    public void intake(double speed)
    {
        intakeMotor.set(speed);
    }

    @Override
    public void periodic() {

    }
        
    }
