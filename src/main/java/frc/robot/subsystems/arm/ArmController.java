package frc.robot.subsystems.arm;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public final class ArmController {
    private final double PI2 = 2 * Math.PI;

    private ProfiledPIDController normalController = new ProfiledPIDController(
        20.0,
        0.0, 
        0.0,
        ArmConstants.kCubeMotionConstraint);

    private TrapezoidProfile.State lastState;

    public ArmController(double measurement) {
        // Note: All inputs must be converted from revolutions to radians

        // Starting arm velocity must be 0
        // loop.reset(VecBuilder.fill(measurement * PI2, 0));

        lastState = new TrapezoidProfile.State(measurement * PI2, 0);

        // System.out.println(observer.getK());
    }

    public double calculate(double measurement, double goal, Constraints constraints) {
        SmartDashboard.putNumber("Trapezoidal pos", lastState.position / PI2);
    
        TrapezoidProfile.State goalState = new TrapezoidProfile.State(goal, 0);
        return normalController.calculate(measurement, goalState, constraints);
    }
}
