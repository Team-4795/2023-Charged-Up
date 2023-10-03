package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autoPaths.*;
import frc.robot.Commands.AutoCommands;

public class AutoSelector {
    private final LoggedDashboardChooser<AutoPath> chooser = new LoggedDashboardChooser<>("Auto Selector");

    AutoCommands autoCommands;

    public AutoSelector() {
        autoCommands = new AutoCommands();

        chooser.addDefaultOption("Free 3 Hybrid MHM", new Free3HybridMHM());

        chooser.addOption("Center 2 + Balance", new Center2CubeBalance());

        chooser.addOption("Center 1 + Balance", new Center1CubeBalance());

        chooser.addOption("Center 1 + Mobility + Balance", new Center1CubeBalanceMobility());

        chooser.addOption("Cable 2 Cube", new Cable2Cube());

        chooser.addOption("Cable 25", new Cable25HybridMHL());
    }

    public Command getSelected() {
        return chooser.get().load(autoCommands);
    }
}
