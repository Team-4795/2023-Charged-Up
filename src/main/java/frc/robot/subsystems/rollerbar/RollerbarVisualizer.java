// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollerbar;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/* Helper class for logging arm state */
public class RollerbarVisualizer {
    String name;
    Mechanism2d mechanism;
    MechanismRoot2d mechanismRoot1;
    MechanismRoot2d mechanismRoot2;

    MechanismLigament2d p1;
    MechanismLigament2d p2;
    MechanismLigament2d p3;

    private final double Bx = Units.inchesToMeters(6.118);
    private final double By = Units.inchesToMeters(0.9);

    private final double ALength = Units.inchesToMeters(13.097);
    private final double BLength = Units.inchesToMeters(10.304);
    private final double ELength = Units.inchesToMeters(7.5);

    private final double EDist = Units.inchesToMeters(4);

    public RollerbarVisualizer(String name, Color color) {
        this.name = name;

        mechanism = new Mechanism2d(2, 1.5);
        mechanismRoot1 = mechanism.getRoot("p1", 0.8 + 0.262, 0.75 - 0.612);
        mechanismRoot2 = mechanism.getRoot("p2", 0.8 + 0.417, 0.75 - 0.589);

        p1 = mechanismRoot1.append(new MechanismLigament2d("p1", ALength, 0, 6, new Color8Bit(color)));
        p2 = mechanismRoot2.append(new MechanismLigament2d("p2", BLength, 0, 6, new Color8Bit(color)));
        p3 = p1.append(new MechanismLigament2d("p3", ELength, 0, 6, new Color8Bit(color)));
    }

    public void update(double p1Angle) {
        double p1AngleRad = Math.toRadians(p1Angle);

        double Cx = ALength * Math.cos(p1AngleRad);
        double Cy = ALength * Math.sin(p1AngleRad);

        double p2AngleRad = solveP2Angle(p1AngleRad, Cx, Cy);
        double p3AngleRad = solveP3Angle(p1AngleRad, p2AngleRad, Cx, Cy);

        p1.setAngle(Rotation2d.fromRadians(p1AngleRad));
        p2.setAngle(Rotation2d.fromRadians(p2AngleRad));
        p3.setAngle(Rotation2d.fromRadians(p3AngleRad - p1AngleRad));

        Logger.getInstance().recordOutput("RollerbarMechanism2d/" + name, mechanism);

        var posep1 = new Pose3d(0.016, 0, 0.155, new Rotation3d(0, -p1AngleRad + Math.toRadians(22.353), 0));
        var posep2 = new Pose3d(0.171, 0, 0.178, new Rotation3d(0, -p2AngleRad + Math.toRadians(17.042), 0));
        var posep3 = new Pose3d(0.016 + Cx, 0, 0.155 + Cy, new Rotation3d(0, -p3AngleRad + Math.toRadians(-15.249), 0));

        Logger.getInstance().recordOutput("RollerbarMechanism3d/" + name, posep1, posep2, posep3);
    }

    public double solveP2Angle(double p1Angle, double Cx, double Cy) {
        return dist(Bx - Cx, By - Cy);
    }

    public double solveP3Angle(double p1Angle, double p2Angle, double Cx, double Cy) {
        double Dx = Bx + BLength * Math.cos(p2Angle);
        double Dy = By + BLength * Math.sin(p2Angle);

        return Math.atan2(Dy - Cy, Dx - Cx);
    }

    public double dist(double a, double b) {
        double d = a * a + b * b;
        double angle = Math.acos((EDist * EDist - (BLength * BLength + d)) / (2 * BLength * Math.sqrt(d)));
        double offset = Math.atan2(b, a);
        return angle + offset;
    }
}
