// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Arm.IDLE_UNDER_STAGE;
import static frc.robot.Constants.Arm.INTAKING_POSITION;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.LoggedDashboardChooser;


public class RobotContainer {

    private final CommandXboxController chassisDriver = new CommandXboxController(0);
    private final CommandXboxController subsystemsDriver = new CommandXboxController(1);



  public RobotContainer() {

    configureBindings();

  }

  private void configureBindings() {

  
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
