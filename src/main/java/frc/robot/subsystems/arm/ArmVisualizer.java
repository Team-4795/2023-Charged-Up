// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/* Helper class for logging arm state */
public class ArmVisualizer {
    String name;
    Mechanism2d mechanism;
    MechanismRoot2d mechanismRoot;
    MechanismLigament2d arm;
    MechanismLigament2d wrist;

    public ArmVisualizer(String name, Color color) {
        this.name = name;

        mechanism = new Mechanism2d(2, 1.5);
        mechanismRoot = mechanism.getRoot("Arm", 0.8, 0.75);
        arm = mechanismRoot.append(new MechanismLigament2d("Arm", 0.5334, 0, 6, new Color8Bit(color)));
        wrist = arm.append(new MechanismLigament2d("Wrist", 0.298, 0, 6, new Color8Bit(color)));
    }

    public void update(double armAngle, double wristAngle) {
        arm.setAngle(armAngle);
        wrist.setAngle(wristAngle);
        Logger.getInstance().recordOutput("ArmMechanism2d/" + name, mechanism);

        var armPose = new Pose3d(-0.246, 0, 0.767, new Rotation3d(0, -Math.toRadians(armAngle), 0));
        var wristPose = armPose.transformBy(new Transform3d(
                new Translation3d(0.5334, 0.0, 0.0), new Rotation3d(0, -Math.toRadians(wristAngle), 0)));

        Logger.getInstance().recordOutput("ArmMechanism3d/" + name, armPose, wristPose);
    }
}
