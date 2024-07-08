// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Arm.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
import frc.robot.Commands.SwerveCommands.DriveCommands;
import frc.robot.Subsystems.Arm.ArmIO;
import frc.robot.Subsystems.Arm.ArmIOSparkMax;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOSparkMax;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOSim;
import frc.robot.Subsystems.Shooter.ShooterIOTalon;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Subsystems.Swerve.GyroIO;
import frc.robot.Subsystems.Swerve.GyroIOPigeon2;
import frc.robot.Subsystems.Swerve.ModuleIO;
import frc.robot.Subsystems.Swerve.ModuleIOSim;
import frc.robot.Subsystems.Swerve.ModuleIOTalonFX;
import frc.robot.Subsystems.Vision.AprilTagIO;
import frc.robot.Subsystems.Vision.AprilTagIOLimelight;
import frc.robot.Subsystems.Vision.AprilTagLocalizer;

public class RobotContainer {

    private final CommandXboxController chassisDriver = new CommandXboxController(0);
    private final CommandXboxController subsystemsDriver = new CommandXboxController(1);

  private static LoggedDashboardChooser<AutoCommand> autoChooser = new LoggedDashboardChooser<>("Auto Mode");

  public static Field2d autoPreviewField = new Field2d();

  public static Drive drive;
  
  public static ShooterIO shooterIO = new ShooterIOTalon();
  public static ShooterSubsystem m_shooter;

  public static IntakeIO intakeIO = new IntakeIOSparkMax();
  public static IntakeSubsystem m_intake = new IntakeSubsystem(intakeIO);

  public static ArmIO armIO = new ArmIOSparkMax();
  public static ArmSubsystem m_arm = new ArmSubsystem(armIO);

  public static AprilTagIO visionIO = new AprilTagIOLimelight();        
  public static AprilTagLocalizer m_vision;

  public RobotContainer() {
  /** Options for the current mode of the robot */
     switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
      drive = new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));   
      m_shooter = new ShooterSubsystem(shooterIO);  
      m_vision = new AprilTagLocalizer(drive, visionIO);  
        break;
        //--------------------------------------------
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_shooter = new ShooterSubsystem(new ShooterIOSim());
        break;
        //--------------------------------------------
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
        //--------------------------------------------
    }
    /** Visualisation of the current auto selected **/
    autoChooser.onChange(
        auto -> {
          autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());});

    /**Auto options */      
    autoChooser.addDefaultOption("None", new NoneAuto());
    autoChooser.addOption("Test1", new Test1());
    autoChooser.addOption("Test2", new Test2());
    
    SmartDashboard.putData("Auto Preview", autoPreviewField);

    configureBindings();
  }

  private void configureBindings() {

  /* Control 1 commands */
    //Chassis commands
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -chassisDriver.getLeftY(),
            () -> -chassisDriver.getLeftX(),
            () -> -chassisDriver.getRightX()));

      //Lock modules
    chassisDriver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    
      //Set field centric
    chassisDriver.a().onTrue(drive.runOnce(() -> drive.zeroHeading()));


    chassisDriver.rightBumper()
    .whileTrue(new IntakeWSensor(m_intake)
    .alongWith(m_arm.goToPosition(INTAKING_POSITION)))
    .whileFalse(m_arm.goToPosition(IDLE_UNDER_STAGE));

    /* Control 2 commands */
    subsystemsDriver.leftBumper()
    .whileTrue(new AmpShoot(m_shooter,m_intake))
    .whileFalse(m_arm.goToPosition(IDLE_UNDER_STAGE));

    subsystemsDriver.rightBumper()
    .whileTrue(new Intake(m_intake));

    subsystemsDriver.x()
    .whileTrue(new SpeakerShoot(m_shooter)
    .alongWith(m_arm.goToPosition(SPEAKER_SCORING_POSITION)))
    .whileFalse(m_arm.goToPosition(IDLE_UNDER_STAGE));

    subsystemsDriver.povLeft()
    .whileTrue(new UnderStageShoot(m_shooter))
    .whileFalse(m_arm.goToPosition(IDLE_UNDER_STAGE));

    subsystemsDriver.povRight()
    .whileTrue(new OverStageShoot(m_shooter));

    subsystemsDriver.b()
    .whileTrue(new Outake(m_intake, m_shooter));

    subsystemsDriver.a()
    .onTrue(m_arm.goToPosition(93));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public ArmSubsystem getArmSubsystem(){
    return m_arm;
  }

}
