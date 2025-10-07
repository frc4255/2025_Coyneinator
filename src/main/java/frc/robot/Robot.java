// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.graph.GraphParser;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
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
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    configureAdvantageKit();

    Logger.start();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    GraphParser.funny();

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_robotContainer.updateManager();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setManagerAsInactive();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private void configureAdvantageKit() {
    Logger.recordMetadata("ProjectName", "Coyneinator");
    Logger.recordMetadata("RuntimeType", isReal() ? "REAL" : "SIM");
    Logger.recordMetadata("RuntimeLocation", Filesystem.getOperatingDirectory().toString());

    if (shouldReplay()) {
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replayed")));
    } else if (isReal()) {
      Path logsDir = Paths.get(Filesystem.getOperatingDirectory().toString(), "logs");
      try {
        Files.createDirectories(logsDir);
      } catch (IOException ignored) {}
      Logger.addDataReceiver(new WPILOGWriter(logsDir.resolve(timestampedLogName("real")).toString()));
    } else {
      // In sim, default to real-time timing to reduce CPU usage and UI lag.
      // Opt-in to fast sim with -Dsim.fast=true or environment SIM_FAST=true
      boolean fastSim = Boolean.getBoolean("sim.fast")
          || "true".equalsIgnoreCase(System.getenv("SIM_FAST"));
      if (fastSim) {
        setUseTiming(false);
      }

      // Prefer live NT4 streaming only; write a .wpilog in sim only if explicitly requested
      boolean writeSimFile = Boolean.getBoolean("ak.logFileSim")
          || "true".equalsIgnoreCase(System.getenv("AK_LOG_FILE_SIM"));
      if (writeSimFile) {
        Path logsDir = Paths.get(Filesystem.getOperatingDirectory().toString(), "logs", "sim");
        try {
          Files.createDirectories(logsDir);
        } catch (IOException ignored) {}
        Logger.addDataReceiver(new WPILOGWriter(logsDir.resolve(timestampedLogName("sim")).toString()));
      }
    }

    Logger.addDataReceiver(new NT4Publisher());
  }

  private static boolean shouldReplay() {
    String property = System.getProperty("advkit.replay");
    if (property != null) {
      return property.equalsIgnoreCase("true");
    }
    String env = System.getenv("ADVANTAGEKIT_REPLAY");
    return env != null && env.equalsIgnoreCase("true");
  }

  private static String timestampedLogName(String prefix) {
    return prefix + "-" + System.currentTimeMillis() + ".wpilog";
  }
}
