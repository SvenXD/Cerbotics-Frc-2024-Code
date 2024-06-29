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
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Commands.AutoCommands.NoneAuto;
import frc.robot.Commands.AutoCommands.Test1;
import frc.robot.Commands.AutoCommands.Test2;
import frc.robot.Commands.IntakeCommands.Intake;
import frc.robot.Commands.IntakeCommands.IntakeWSensor;
import frc.robot.Commands.IntakeCommands.Outake;
import frc.robot.Commands.ShooterCommands.AmpShoot;
import frc.robot.Commands.ShooterCommands.OverStageShoot;
import frc.robot.Commands.ShooterCommands.SpeakerShoot;
import frc.robot.Commands.ShooterCommands.UnderStageShoot;
import frc.robot.Subsystems.Arm.ArmIO;
import frc.robot.Subsystems.Arm.ArmIOSparkMax;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOSparkMax;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOTalon;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {

    private final CommandXboxController chassisDriver = new CommandXboxController(0);
    private final CommandXboxController subsystemsDriver = new CommandXboxController(1);

  private static LoggedDashboardChooser<AutoCommand> autoChooser;
  public static Field2d autoPreviewField = new Field2d();

  public static ShooterIO shooterIO = new ShooterIOTalon();
  public static ShooterSubsystem m_shooter = new ShooterSubsystem(shooterIO);

  public static IntakeIO intakeIO = new IntakeIOSparkMax();
  public static IntakeSubsystem m_intake = new IntakeSubsystem(intakeIO);

  public static ArmIO armIO = new ArmIOSparkMax();
  public static ArmSubsystem m_arm = new ArmSubsystem(armIO);


  public RobotContainer() {

    autoChooser = new LoggedDashboardChooser<>("Auto Mode");

    autoChooser.onChange(
        auto -> {
          autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());});

           autoChooser.addDefaultOption("None", new NoneAuto());
    autoChooser.addOption("Test1", new Test1());
    autoChooser.addOption("Test2", new Test2());
    
    SmartDashboard.putData("Auto Preview", autoPreviewField);

    configureBindings();

  }

  private void configureBindings() {

    chassisDriver.rightBumper()
    .whileTrue(new IntakeWSensor(m_intake)
    .alongWith(m_arm.goToPosition(INTAKING_POSITION)))
    .whileFalse(m_arm.goToPosition(IDLE_UNDER_STAGE));

    subsystemsDriver.leftBumper()
    .whileTrue(new AmpShoot(m_shooter));

    subsystemsDriver.rightBumper()
    .whileTrue(new Intake(m_intake));

    subsystemsDriver.x()
    .whileTrue(new SpeakerShoot(m_shooter));

    subsystemsDriver.povLeft()
    .whileTrue(new UnderStageShoot(m_shooter));

    subsystemsDriver.povRight()
    .whileTrue(new OverStageShoot(m_shooter));

    subsystemsDriver.b()
    .whileTrue(new Outake(m_intake, m_shooter));

    subsystemsDriver.a()
    .onTrue(m_arm.goToPosition(150));

    subsystemsDriver.y()
    .onTrue(m_arm.goToPosition(110));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public ArmSubsystem getArmSubsystem(){
    return m_arm;
  }
}
