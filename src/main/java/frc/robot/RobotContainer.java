// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Arm.IDLE_UNDER_STAGE;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.LoggedDashboardChooser;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Subsystems.Swerve.GyroIOPigeon2;
import frc.robot.Subsystems.Swerve.ModuleIO;
import frc.robot.Subsystems.Swerve.ModuleIOKraken;
import frc.robot.Subsystems.Swerve.Module;


public class RobotContainer {

  private Drive drive;
    private final CommandXboxController chassisDriver = new CommandXboxController(0);
    private final CommandXboxController subsystemsDriver = new CommandXboxController(1);



  public RobotContainer() {

     drive =
              new Drive(
                  new ModuleIOKraken(DriveConstants.moduleConfigs[0]),
                  new ModuleIOKraken(DriveConstants.moduleConfigs[1]),
                  new ModuleIOKraken(DriveConstants.moduleConfigs[2]),
                  new ModuleIOKraken(DriveConstants.moduleConfigs[3]));

    configureBindings();

  }

  private void configureBindings() {

    drive.setDefaultCommand(
      drive
          .run(
              () ->
                  drive.acceptTeleopInput(
                      -chassisDriver.getLeftY(),
                      -chassisDriver.getLeftX(),
                      -chassisDriver.getRightX(),
                      false))
          .withName("Drive Teleop Input"));
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

