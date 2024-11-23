// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.CTRE.swerve.SwerveModule.DriveRequestType;
import frc.Util.CTRE.swerve.SwerveRequest;
import frc.Util.CTRE.swerve.SwerveRequest.ForwardReference;
import frc.Util.LocalADStarAK;
import frc.Util.Logging.LoggedDashboardChooser;
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Commands.AutoCommands.Paths.NoneAuto;
import frc.robot.Commands.AutoCommands.Paths.RegionalPaths.MoveTest;
import frc.robot.Commands.IntakeCommands.IntakeWithSensor;
import frc.robot.Commands.ShooterCommands.ShooterCommand;
import frc.robot.Commands.SwerveCommands.FieldCentricDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Arm.ArmIO;
import frc.robot.Subsystems.Arm.ArmIOKraken;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Climber.ClimberIO;
import frc.robot.Subsystems.Climber.ClimberIOKraken;
import frc.robot.Subsystems.Climber.ClimberSubsystem;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOKraken;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOTalon;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Swerve.CTRESwerve.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Swerve.CTRESwerve.TunerConstants;
import frc.robot.Subsystems.Vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  public static final CommandXboxController chassisDriver = new CommandXboxController(0);
  public static final CommandXboxController subsystemsDriver = new CommandXboxController(1);

  private static LoggedDashboardChooser<AutoCommand> autoChooser;

  public static Field2d autoPreviewField = new Field2d();

  public static ArmIO armIO = new ArmIOKraken();
  public static ArmSubsystem m_arm = new ArmSubsystem(armIO);

  public static IntakeIO intakeIO = new IntakeIOKraken();
  public static IntakeSubsystem m_intake = new IntakeSubsystem(intakeIO);

  public static ShooterIO shooterIO = new ShooterIOTalon();
  public static ShooterSubsystem m_shooter = new ShooterSubsystem(shooterIO);

  public static ClimberIO climberIO = new ClimberIOKraken();
  public static ClimberSubsystem m_climber = new ClimberSubsystem(climberIO);

  private static final CommandSwerveDrivetrain m_drive = TunerConstants.DriveTrain;

  SwerveRequest.FieldCentricFacingAngle m_head =
      new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity);

  private static final VisionSubsystem m_vision = new VisionSubsystem(m_drive, "1");

  private final Telemetry logger = new Telemetry(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

  public RobotContainer() {

    m_head.ForwardReference = ForwardReference.RedAlliance;
    m_head.HeadingController.setPID(8, 0, 0);
    m_head.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    /** Options for the current mode of the robot */
    /*switch (Constants.currentMode) {
         case REAL:
           // Real robot, instantiate hardware IO implementations
           drive =
               new Drive(
                   new GyroIOPigeon2(true),
                   new ModuleIOTalonFX(0),
                   new ModuleIOTalonFX(1),
                   new ModuleIOTalonFX(2),
                   new ModuleIOTalonFX(3));

           break;
           // --------------------------------------------
         case SIM:
           // Sim robot, instantiate physics sim IO implementations
           drive =
               new Drive(
                   new GyroIO() {},
                   new ModuleIOSim(),
                   new ModuleIOSim(),
                   new ModuleIOSim(),
                   new ModuleIOSim());

           break;
           // --------------------------------------------
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
           // --------------------------------------------
       }
    */
    registerNamedCommands();
    /** Visualisation of the current auto selected * */
    autoChooser = new LoggedDashboardChooser<>("Auto Mode");

    autoChooser.onChange(
        auto -> {
          autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
        });

    /* Auto options */
    autoChooser.addDefaultOption("None", new NoneAuto());
    autoChooser.addOption("4 Note Auto", new MoveTest());

    PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

    SmartDashboard.putData("Auto Preview", autoPreviewField);

    SmartDashboard.putString("Current Robot mode", Constants.currentMode.toString());

    Pathfinding.setPathfinder(new LocalADStarAK());

    configureBindings();

    m_drive.registerTelemetry(logger::telemeterize);
  }

  private void configureBindings() {

    /* Driver 1 */
    m_drive.setDefaultCommand(
        new FieldCentricDrive(
            m_drive,
            () -> -chassisDriver.getLeftY() * filterSpeed(),
            () -> -chassisDriver.getLeftX() * filterSpeed(),
            () -> -chassisDriver.getRightX()));

    chassisDriver.a().onTrue(m_drive.runOnce(() -> m_drive.seedFieldRelative()));

    chassisDriver
        .rightBumper()
        .whileTrue(m_arm.goToPosition(172).alongWith(new IntakeWithSensor(m_intake)))
        .whileFalse(m_arm.goToPosition(172));

    chassisDriver
        .rightBumper()
        .and(() -> m_intake.isNoteInside())
        .onTrue(controllerRumbleCommand().withTimeout(1));

    chassisDriver
        .b()
        .whileTrue(m_shooter.setRpms(-10, -10).alongWith(m_intake.setall(0.5, 0.5)))
        .whileFalse(m_intake.setall(0, 0).alongWith(m_shooter.stop()));

    /* Driver 2 */
    subsystemsDriver
        .a()
        .onTrue(
            m_arm
                .goToPosition(95)
                .alongWith(m_shooter.setRpms(11, 11).onlyIf((() -> m_arm.getArmAngle() < 180))));

    subsystemsDriver
        .x()
        .whileTrue(m_shooter.setRpms(90, 90).alongWith(m_arm.goToPosition(167)))
        .whileFalse(m_arm.goToPosition(172).alongWith(m_shooter.stop()));

    subsystemsDriver
        .b()
        .whileTrue(m_shooter.setRpms(90, 90).alongWith(m_arm.goToPosition(115)))
        .whileFalse(m_arm.goToPosition(172).alongWith(m_shooter.stop()));

    subsystemsDriver
        .rightBumper()
        .whileTrue(m_intake.setUpperVoltage(-1))
        .whileFalse(m_intake.setUpperVoltage(0));

    subsystemsDriver
        .leftBumper()
        .whileTrue(m_intake.setall(0.5, 0.5).alongWith(m_shooter.setRpms(-2, -2)))
        .whileFalse(
            m_intake.setall(0, 0).alongWith(m_shooter.stop()).alongWith(m_arm.goToPosition(172)));

    subsystemsDriver
        .povDown()
        .whileTrue(m_climber.setClimberVoltage(1))
        .whileFalse(m_climber.setClimberVoltage(0));

    subsystemsDriver.povUp().onTrue(m_climber.setPosition(111));
  }

  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          chassisDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          subsystemsDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          chassisDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          subsystemsDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  private double filterSpeed() {
    double val = 1;
    if (chassisDriver.getRightX() > 5) {
      val = 0.1;
    }
    return val;
  }

  /*public static Command pathfindAndAlignAmp() {
        return Commands.either(
            drive
                .goToPose(FieldConstants.redAmpPose)
                .until(() -> Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
            drive
                .goToPose(FieldConstants.blueAmpPose)
                .until(() -> Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
            Robot::isRedAlliance);
      }

      public static Command pathfindAndAlignSource() {
        return Commands.either(
            drive
                .goToPose(FieldConstants.redPickupPose)
                .until(() -> Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
            drive
                .goToPose(FieldConstants.bluePickupPose)
                .until(() -> Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
            Robot::isRedAlliance);
      }
  */
  public void registerNamedCommands() {
    NamedCommands.registerCommand("Arm169", m_arm.goToPosition(167));
    NamedCommands.registerCommand(
        "FirstShoot", new ShooterCommand(m_shooter, m_intake).alongWith(m_arm.goToPosition(169)));

    NamedCommands.registerCommand(
        "Intake", new IntakeWithSensor(m_intake).alongWith(m_arm.goToPosition(172)));

    NamedCommands.registerCommand(
        "PrepareShoot", m_shooter.setRpms(90, 90).until(() -> !m_intake.isNoteInside()));
    NamedCommands.registerCommand(
        "IntakeToShoot", m_intake.setall(-0.9, -1).until(() -> !m_intake.isNoteInside()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public CommandSwerveDrivetrain getDrive() {
    return m_drive;
  }

  public static ArmSubsystem getArm() {
    return m_arm;
  }

  public static ShooterSubsystem getShooter() {
    return m_shooter;
  }

  public static IntakeSubsystem getIntake() {
    return m_intake;
  }
}
