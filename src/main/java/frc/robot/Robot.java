// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.arm.Arm;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private static long teleopStart;
    private double m_rumble = 0;
    private static boolean isTeleOp = true;

    public static double getSeconds() {
        return 135.0 - (System.currentTimeMillis() - teleopStart) / 1000.0; // change for testing
    }

    public static boolean isTeleOp() {
        return isTeleOp;
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger logger = Logger.getInstance();
        switch (Constants.getRobot()) {
            case Comp:
              logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
              logger.addDataReceiver(new NT4Publisher());
              // LoggedPowerDistribution.getInstance(50, ModuleType.kRev);
              break;
      
            // Running a physics simulator, log to local folder
            case Sim:
              // logger.addDataReceiver(new WPILOGWriter(""));
              logger.addDataReceiver(new NT4Publisher());
              break;
      
            // Replaying a log, set up replay source
            case Replay:
              setUseTiming(false); // Run as fast as possible
              String logPath = LogFileUtil.findReplayLog();
              logger.setReplaySource(new WPILOGReader(logPath));
              logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
              break;
        }
        logger.start();

        if (Constants.getRobot() == RobotType.Sim) {
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        }

        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Log active commands
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.getInstance()
                    .recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.getInstance().recordOutput("CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
            logCommandFunction.accept(command, true);
        });
        CommandScheduler.getInstance().onCommandFinish((Command command) -> {
            logCommandFunction.accept(command, false);
        });
        CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
            logCommandFunction.accept(command, false);
        });

        // PathPlannerServer.startServer(5811); // 4795 = port number
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());



        //Limelight port forwarding
        for(int port = 5000; port <= 5805; port++){
            PortForwarder.add(port, "limelight.local", port);
        }

    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        VirtualSubsystem.periodicAll();
        CommandScheduler.getInstance().run();
        Threads.setCurrentThreadPriority(true, 10);
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        isTeleOp = false;

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        teleopStart = System.currentTimeMillis();
        isTeleOp = true;
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // m_robotContainer.setNotStoring();
        Arm.getInstance().resetPosition();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        isTeleOp = true;
        if (getSeconds() <= 30 && getSeconds() >= 28) {
            m_rumble = 0.5;
            m_robotContainer.setRumble(0.25);
        } else if (getSeconds() <= 15 && getSeconds() >= 13) {
            m_rumble = 0.5;
            m_robotContainer.setRumble(0.25);
        } else {
            m_rumble = 0;
            m_robotContainer.setRumble(0);
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
