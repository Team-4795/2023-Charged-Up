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
        40.0,
        0.0, 
        0.0,
        ArmConstants.kCubeMotionConstraint);

    private TrapezoidProfile.State lastState;

    private final LinearSystem<N2, N1, N1> armPlant =
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2), ArmConstants.kArmMOI, ArmConstants.kGearing);

    private final KalmanFilter<N2, N1, N1> observer =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            armPlant,
            VecBuilder.fill(0.5, 0.7), // How accurate we
            // think our model is, in radians and radians/sec
            VecBuilder.fill(0.05), // How accurate we think our encoder position
            // data is. In this case we very highly trust our encoder position reading.
            Constants.DT);

    private final LinearQuadraticRegulator<N2, N1, N1> lqr =
        new LinearQuadraticRegulator<>(
            armPlant,
            VecBuilder.fill(0.05, 1), // Position, Velocity weight (Lower is more penalized)
            VecBuilder.fill(12.0), // Voltage weight
            Constants.DT);

    private final LinearSystemLoop<N2, N1, N1> loop =
        new LinearSystemLoop<>(armPlant, lqr, observer, 12.0, Constants.DT);

    public ArmController(double measurement) {
        // Note: All inputs must be converted from revolutions to radians

        // Starting arm velocity must be 0
        loop.reset(VecBuilder.fill(measurement * PI2, 0));

        lastState = new TrapezoidProfile.State(measurement * PI2, 0);

        System.out.println(observer.getK());
    }

    public double calculate(double measurement, double goal, Constraints constraints) {
        SmartDashboard.putNumber("Trapezoidal pos", lastState.position / PI2);

        if (Constants.USE_LQR) {
            SmartDashboard.putNumber("Arm estimate pos", observer.getXhat(0) / PI2);
            SmartDashboard.putNumber("Arm estimate vel", observer.getXhat(1) / PI2);

            // Map values to radians
            constraints = new Constraints(constraints.maxVelocity * PI2, constraints.maxAcceleration * PI2);
            measurement *= PI2;
            goal *= PI2;

            TrapezoidProfile prof = new TrapezoidProfile(constraints, new TrapezoidProfile.State(goal, 0), lastState);
            lastState = prof.calculate(Constants.DT);

            loop.setNextR(lastState.position, lastState.velocity);
            loop.correct(VecBuilder.fill(measurement));
            loop.predict(Constants.DT);

            return loop.getU(0);
        } else {
            TrapezoidProfile.State goalState = new TrapezoidProfile.State(goal, 0);
            return normalController.calculate(measurement, goalState, constraints);
        }
    }
}
