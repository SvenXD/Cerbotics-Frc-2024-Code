// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
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
import frc.Util.LocalADStarAK;
import frc.Util.Logging.LoggedDashboardChooser;
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Commands.AutoCommands.Paths.NoneAuto;
import frc.robot.Commands.AutoCommands.Paths.RegionalPaths.MoveTest;
import frc.robot.Commands.IntakeCommands.IntakeWithSensor;
import frc.robot.Commands.ShooterCommands.ShooterCommand;
import frc.robot.Commands.SwerveCommands.FieldCentricDrive;
import frc.robot.Subsystems.Arm.ArmIO;
import frc.robot.Subsystems.Arm.ArmIOKraken;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOKraken;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOTalon;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Swerve.CTRESwerve.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Swerve.CTRESwerve.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  // nopublic static SimDefenseBot defenseBot = new SimDefenseBot(2);

  private static final CommandXboxController chassisDriver = new CommandXboxController(0);
  private static final CommandXboxController subsystemsDriver = new CommandXboxController(1);

  private static LoggedDashboardChooser<AutoCommand> autoChooser;

  public static Field2d autoPreviewField = new Field2d();

  // public static Drive drive;

  public static ArmIO armIO = new ArmIOKraken();
  public static ArmSubsystem m_arm = new ArmSubsystem(armIO);

  public static IntakeIO intakeIO = new IntakeIOKraken();
  public static IntakeSubsystem m_intake = new IntakeSubsystem(intakeIO);

  public static ShooterIO shooterIO = new ShooterIOTalon();
  public static ShooterSubsystem m_shooter = new ShooterSubsystem(shooterIO);

  // public static ClimberIO climberIO = new ClimberIOKraken();
  // public static ClimberSubsystem m_climber = new ClimberSubsystem(climberIO);

  private static final CommandSwerveDrivetrain m_drive = TunerConstants.DriveTrain;

  SwerveRequest.FieldCentricFacingAngle m_head =
      new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity);

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

    // Set up note visualizer

    configureBindings();
  }

  private void configureBindings() {

    /* Driver 1 */
    m_drive.setDefaultCommand(
        new FieldCentricDrive(
            m_drive,
            () -> -chassisDriver.getLeftY() * 0.3,
            () -> -chassisDriver.getLeftX() * 0.3,
            () -> -chassisDriver.getRightX() * 0.6));

    chassisDriver.a().onTrue(m_drive.runOnce(() -> m_drive.seedFieldRelative()));

    chassisDriver
        .rightBumper()
        .whileTrue(m_arm.goToPosition(173).alongWith(new IntakeWithSensor(m_intake)))
        .whileFalse(m_arm.goToPosition(173));

    chassisDriver
        .rightBumper()
        .and(() -> m_intake.isNoteInside())
        .onTrue(controllerRumbleCommand().withTimeout(1));

    /* Driver 2 */
    subsystemsDriver
        .a()
        .onTrue(
            m_arm
                .goToPosition(95)
                .alongWith(m_shooter.setRpms(70, 70).onlyIf((() -> m_arm.getArmAngle() < 180))));

    subsystemsDriver
        .x()
        .whileTrue(m_shooter.setRpms(90, 90).alongWith(m_arm.goToPosition(173)))
        .whileFalse(m_arm.goToPosition(173).alongWith(m_shooter.stop()));

    subsystemsDriver
        .leftBumper()
        .whileTrue(m_intake.setUpperVoltage(-1))
        .whileFalse(
            m_shooter
                .stop()
                .alongWith(m_arm.goToPosition(173))
                .alongWith(m_intake.setUpperVoltage(0)));

    subsystemsDriver
        .rightBumper()
        .whileTrue(m_intake.setUpperVoltage(-1))
        .whileFalse(m_intake.setUpperVoltage(0));

    subsystemsDriver
        .b()
        .whileTrue(m_intake.setall(0.5, 0.5).alongWith(m_shooter.setRpms(-2, -2)))
        .whileFalse(m_intake.setall(0, 0).alongWith(m_shooter.stop()));
    // 1200 rpm

    /*subsystemsDriver
        .b()
        .whileTrue(m_intake.setlowerIntakeVoltage(0.3))
        .whileFalse(m_intake.setlowerIntakeVoltage(0));

    subsystemsDriver
        .x()
        .whileTrue(m_intake.setall(-0.7, -0.8)) // Intake
        .whileFalse(m_intake.setall(0, 0));

    subsystemsDriver
        .povUp()
        .whileTrue(m_climber.setClimberVoltage(0.1))
        .whileFalse(m_climber.setClimberVoltage(0));

    /*subsystemsDriver.x().whileTrue(m_shooter.setRpms(100, 100)).whileFalse(m_shooter.stop());
    subsystemsDriver.b().whileTrue(m_shooter.setPower()).whileFalse(m_shooter.stop());


    -0.7 intake for all  0.8 for the one ont he right*/
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
    NamedCommands.registerCommand("Arm160", m_arm.goToPosition(160));
    NamedCommands.registerCommand("FirstShoot", new ShooterCommand(m_shooter, m_intake));
    NamedCommands.registerCommand(
        "Intake", new IntakeWithSensor(m_intake).alongWith(m_arm.goToPosition(174)));
    NamedCommands.registerCommand(
        "PrepareShoot", m_shooter.setRpms(90, 90).until(() -> !m_intake.isNoteInside()));
    NamedCommands.registerCommand(
        "IntakeToShoot", m_intake.setall(-0.9, -1).until(() -> !m_intake.isNoteInside()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static CommandSwerveDrivetrain getDrive() {
    return m_drive;
  }

  public static ArmSubsystem getArm() {
    return m_arm;
  }

  public static ShooterSubsystem getShooter() {
    return m_shooter;
  }
}
