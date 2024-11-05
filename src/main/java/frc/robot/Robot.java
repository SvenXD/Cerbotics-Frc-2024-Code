// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Util.LocalADStarAK;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Command stopShooter;
  private static RobotContainer m_robotContainer;
  StructArrayPublisher<SwerveModuleState> measuredStates;
  StructArrayPublisher<SwerveModuleState> targetStates;

  @Override
  public void robotInit() {
    if (isReal()) {
      Constants.currentMode = Mode.REAL;
    }

    m_robotContainer = new RobotContainer();

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set a metadata value
    Pathfinding.setPathfinder(new LocalADStarAK());
    DataLog log = DataLogManager.getLog();

    if (Constants.needToLog) {
      DataLogManager.start();
      DriverStation.startDataLog(log);
    }

    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        SmartDashboard.putString("test", logPath);

        break;
    }

    measuredStates =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Measured Swerve States", SwerveModuleState.struct)
            .publish();

    targetStates =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Target Swerve States", SwerveModuleState.struct)
            .publish();
    Logger.start();
    Logger.disableDeterministicTimestamps();
    Logger.disableConsoleCapture();
  }

  @Override
  public void robotPeriodic() {
    measuredStates.set(RobotContainer.getDrive().getState().ModuleStates);
    targetStates.set(RobotContainer.getDrive().getState().ModuleTargets);
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("IsRedAlliance", isRedAlliance());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopInit() {
    stopShooter = RobotContainer.getShooter().stop();
    stopShooter.schedule();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {
    Logger.recordOutput("SwervePose", RobotContainer.getDrive().getState().Pose);

    SmartDashboard.putNumber("MatchTime", Timer.getMatchTime());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance()
        .filter(value -> value == DriverStation.Alliance.Red)
        .isPresent();
  }
}
