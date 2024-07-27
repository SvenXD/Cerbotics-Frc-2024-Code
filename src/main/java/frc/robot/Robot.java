// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.LocalADStarAK;
import frc.Util.NoteVisualizer;
import frc.robot.Subsystems.Arm.ArmSubsystem.ArmStates;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private static double[] xNotes = new double[7];
  private static double[] yNotes = new double[7];
  private double[] robotCords = new double[2];

  @Override
  public void robotInit() {

     m_robotContainer = new RobotContainer();
    Logger.recordMetadata("ProjectName", "2024-Beta"); // Set a metadata value
    Pathfinding.setPathfinder(new LocalADStarAK());
    DataLog log = DataLogManager.getLog();

      if(Constants.needToLog){
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
      Logger.addDataReceiver(new NT4Publisher());
      break;

    case REPLAY:
      // Replaying a log, set up replay source
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      break;
  }

  Logger.start();
  Logger.disableDeterministicTimestamps();
  Logger.disableConsoleCapture();


}

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("IsRedAlliance", isRedAlliance());

    robotCords[0] = RobotContainer.getSwerveSubsystem().getPose().getX();
    robotCords[1] = RobotContainer.getSwerveSubsystem().getPose().getY();

    SmartDashboard.putNumber("Xnote", xNotes[0]);
    SmartDashboard.putNumber("Ynote", yNotes[0]);
    SmartDashboard.putBoolean("HasNoteInSim", NoteVisualizer.hasSimNote());

    SmartDashboard.putNumberArray("Robot Coords", robotCords);
    NoteVisualizer.showIntakedNotes(RobotContainer.getArmSubsystem().getAngleRadiants());

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
    NoteVisualizer.resetAutoNotes();

    for(int i = 0; i < 7; i++){
      xNotes[i] = NoteVisualizer.getAutoNote(i).getX();
      yNotes[i] = NoteVisualizer.getAutoNote(i).getY();
    }

    NoteVisualizer.deleteNote(3);
    deleteCords(3);


  }

  @Override
  public void autonomousPeriodic() {
    NoteVisualizer.showAutoNotes();

    for(int i = 0; i < 7; i++){
      if(Math.abs(xNotes[i] - robotCords[0]) < 0.5 &&
         Math.abs(yNotes[i] - robotCords[1]) < 0.5 &&
         RobotContainer.getArmSubsystem().getState() == ArmStates.INTAKING )
         {
        NoteVisualizer.takeAutoNote(i);
        NoteVisualizer.enableShowNote();
      }
    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
     NoteVisualizer.clearAutoNotes();
  }

  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());

    NoteVisualizer.teleopNote();
      if(Math.abs(NoteVisualizer.getSourceNote().getX() - robotCords[0]) < 0.7
         && Math.abs(NoteVisualizer.getSourceNote().getY() - robotCords[1]) < 0.7
         && RobotContainer.getArmSubsystem().getState() == ArmStates.INTAKING)
         {
            NoteVisualizer.enableShowNote();
         }
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
  public void simulationPeriodic(){}

  
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance()
        .filter(value -> value == DriverStation.Alliance.Red)
        .isPresent();
  }

  public static void deleteCords(int note){
    xNotes[note] = 0;
    yNotes[note] = 0;
  }
}