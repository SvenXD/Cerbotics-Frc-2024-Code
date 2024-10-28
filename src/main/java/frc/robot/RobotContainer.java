// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Arm.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
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
import frc.robot.Commands.IntakeCommands.IntakeWithSensor;
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

  // private static LoggedDashboardChooser<AutoCommand> autoChooser;

  public static Field2d autoPreviewField = new Field2d();

  // public static Drive drive;

  public static ArmIO armIO = new ArmIOKraken();
  public static ArmSubsystem m_arm = new ArmSubsystem(armIO);

  public static IntakeIO intakeIO = new IntakeIOKraken();
  public static IntakeSubsystem m_intake = new IntakeSubsystem(intakeIO);

  public static ShooterIO shooterIO = new ShooterIOTalon();
  public static ShooterSubsystem m_shooter = new ShooterSubsystem(shooterIO);

  private final CommandSwerveDrivetrain m_drive = TunerConstants.DriveTrain;

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

    // registerNamedCommands();
    /** Visualisation of the current auto selected * */
    /*autoChooser = new LoggedDashboardChooser<>("Auto Mode");

    autoChooser.onChange(
        auto -> {
          autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
        });

    /** Auto options *
    autoChooser.addDefaultOption("None", new NoneAuto());
    autoChooser.addOption("Complement auto", new ComplementPath());
    autoChooser.addOption("Six Note Auto", new FiveNoteAutoPath());
    // autoChooser.addOption("Test Of Change", new ChangeTest(drive));
    autoChooser.addOption("5 note auto align", new TestAuto()); */

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

    /* Control 1 commands */
    // Chassis commands
    /*drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -chassisDriver.getLeftY(),
            () -> -chassisDriver.getLeftX(),
            () -> -chassisDriver.getRightX() * 0.5));

    // Lock modules
    chassisDriver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Set field centric
    chassisDriver.a().onTrue(drive.runOnce(() -> drive.zeroHeading()));

    chassisDriver.leftBumper().whileTrue(new NoteAlignCommand(drive));*/

    m_drive.setDefaultCommand(
        new FieldCentricDrive(
            m_drive,
            () -> -chassisDriver.getLeftY(),
            () -> -chassisDriver.getLeftX(),
            () -> -chassisDriver.getRightX()));

    //    chassisDriver.y().onTrue(new InstantCommand(() -> drive.changeIntakeAssist()));

    subsystemsDriver.y().onTrue(m_arm.goToPosition(100));
    subsystemsDriver.a().onTrue(m_arm.goToPosition(174));

    subsystemsDriver.leftBumper().whileTrue(m_shooter.setRpms(91, 91)).whileFalse(m_shooter.stop());
    // 1200 rpm
    subsystemsDriver.rightBumper().whileTrue(new IntakeWithSensor(m_intake));

    subsystemsDriver
        .b()
        .whileTrue(m_intake.setlowerIntakeVoltage(0.3))
        .whileFalse(m_intake.setlowerIntakeVoltage(0));

    subsystemsDriver
        .x()
        .whileTrue(m_intake.setlowerIntakeVoltage(-0.3))
        .whileFalse(m_intake.setlowerIntakeVoltage(0));

    /*subsystemsDriver.x().whileTrue(m_shooter.setRpms(100, 100)).whileFalse(m_shooter.stop());
    subsystemsDriver.b().whileTrue(m_shooter.setPower()).whileFalse(m_shooter.stop());*/
  }

  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          chassisDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          chassisDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
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

    public void registerNamedCommands() {
      NamedCommands.registerCommand(
          "ActivateIntakeAssist", new InstantCommand(() -> drive.changeIntakeAssist()));
    }
  */
  public Command getAutonomousCommand() {
    return null;
  }

  /*
  public static Drive getSwerveSubsystem() {
    return drive;
  }*/
}
