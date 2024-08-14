// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Arm.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.LocalADStarAK;
import frc.Util.Logging.LoggedDashboardChooser;
import frc.Util.NoteVisualizer;
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Commands.AutoCommands.GoToNoteCommand;
import frc.robot.Commands.AutoCommands.Paths.ChangeTest;
import frc.robot.Commands.AutoCommands.Paths.ComplementPath;
import frc.robot.Commands.AutoCommands.Paths.FiveNoteAutoPath;
import frc.robot.Commands.AutoCommands.Paths.NoneAuto;
import frc.robot.Commands.AutoCommands.Paths.TestAuto;
import frc.robot.Commands.IntakeCommands.Intake;
import frc.robot.Commands.IntakeCommands.IntakeWSensor;
import frc.robot.Commands.IntakeCommands.Outake;
import frc.robot.Commands.ShooterCommands.AmpShoot;
import frc.robot.Commands.ShooterCommands.OverStageShoot;
import frc.robot.Commands.ShooterCommands.SpeakerShoot;
import frc.robot.Commands.ShooterCommands.UnderStageShoot;
import frc.robot.Commands.SwerveCommands.DriveCommands;
import frc.robot.Commands.SwerveCommands.NoteAlignCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.Arm.ArmIO;
import frc.robot.Subsystems.Arm.ArmIOSim;
import frc.robot.Subsystems.Arm.ArmIOSparkMax;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.ArmSubsystem.ArmStates;
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
import frc.robot.Subsystems.Vision.PhotonAprilTagVision;
import frc.robot.Subsystems.Vision.PhotonSim;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  // nopublic static SimDefenseBot defenseBot = new SimDefenseBot(2);

  private static final CommandXboxController chassisDriver = new CommandXboxController(0);
  private static final CommandXboxController subsystemsDriver = new CommandXboxController(1);

  private static LoggedDashboardChooser<AutoCommand> autoChooser;

  public static Field2d autoPreviewField = new Field2d();

  public static Drive drive;

  public static ShooterIO shooterIO = new ShooterIOTalon();
  public static ShooterSubsystem m_shooter;

  public static IntakeIO intakeIO = new IntakeIOSparkMax();
  public static IntakeSubsystem m_intake = new IntakeSubsystem(intakeIO);

  public static ArmIO armIO = new ArmIOSparkMax();
  public static ArmSubsystem m_arm;

  public static PhotonSim frontLeftCamera;
  public static PhotonSim frontRightCamera;
  public static PhotonSim backLeftCamera;
  public static PhotonSim backRightCamera;

  public static PhotonAprilTagVision aprilTagVision;

  public RobotContainer() {
    /** Options for the current mode of the robot */
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        m_shooter = new ShooterSubsystem(shooterIO);
        m_arm = new ArmSubsystem(armIO);
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
        m_shooter = new ShooterSubsystem(new ShooterIOSim());
        m_arm = new ArmSubsystem(new ArmIOSim());
        frontLeftCamera = new PhotonSim(0);
        frontRightCamera = new PhotonSim(1);
        backLeftCamera = new PhotonSim(2);
        backRightCamera = new PhotonSim(3);

        aprilTagVision =
            new PhotonAprilTagVision(
                drive, frontLeftCamera, frontRightCamera, backLeftCamera, backRightCamera);
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
        m_shooter = new ShooterSubsystem(new ShooterIO() {});
        m_arm = new ArmSubsystem(new ArmIO() {});
        frontLeftCamera = new PhotonSim(0);
        frontRightCamera = new PhotonSim(2);
        backLeftCamera = new PhotonSim(3);
        backRightCamera = new PhotonSim(1);

        aprilTagVision = new PhotonAprilTagVision(drive, frontLeftCamera);

        break;
        // --------------------------------------------
    }

    registerNamedCommands();
    /** Visualisation of the current auto selected * */
    autoChooser = new LoggedDashboardChooser<>("Auto Mode");

    autoChooser.onChange(
        auto -> {
          autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
        });

    /** Auto options */
    autoChooser.addDefaultOption("None", new NoneAuto());
    autoChooser.addOption("Complement auto", new ComplementPath());
    autoChooser.addOption("Six Note Auto", new FiveNoteAutoPath());
    autoChooser.addOption("Test Of Change", new ChangeTest(drive));
    autoChooser.addOption("Test", new TestAuto());

    PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

    SmartDashboard.putData("Auto Preview", autoPreviewField);

    SmartDashboard.putString("Current Robot mode", Constants.currentMode.toString());

    Pathfinding.setPathfinder(new LocalADStarAK());

    // Set up note visualizer
    NoteVisualizer.setRobotPoseSupplier(drive::getPose);

    configureBindings();
  }

  private void configureBindings() {

    /* Control 1 commands */
    // Chassis commands
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -chassisDriver.getLeftY(),
            () -> -chassisDriver.getLeftX(),
            () -> -chassisDriver.getRightX()));

    // Lock modules
    chassisDriver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Set field centric
    chassisDriver.a().onTrue(drive.runOnce(() -> drive.zeroHeading()));

    // AutoRoutines
    chassisDriver.povUp().toggleOnTrue(pathfindAndAlignAmp());

    chassisDriver.povLeft().toggleOnTrue(pathfindAndAlignSource());

    chassisDriver
        .rightBumper()
        .whileTrue(
            new IntakeWSensor(m_intake)
                .alongWith(
                    m_arm.goToPosition(INTAKING_POSITION, m_arm.changeState(ArmStates.INTAKING))))
        .whileFalse(m_arm.goToPosition(IDLE_UNDER_STAGE, m_arm.changeState(ArmStates.IDLE)));

    // Control rumbles when game piece is detected
    chassisDriver
        .rightBumper()
        .and(() -> NoteVisualizer.hasSimNote())
        .onTrue(controllerRumbleCommand().withTimeout(1));

    chassisDriver.leftBumper().whileTrue(new NoteAlignCommand(drive));

    /* Control 2 commands */
    subsystemsDriver
        .leftBumper()
        .whileTrue(new AmpShoot(m_shooter, m_intake).alongWith(NoteVisualizer.ampShoot()))
        .whileFalse(m_arm.goToPosition(IDLE_UNDER_STAGE, m_arm.changeState(ArmStates.IDLE)));

    subsystemsDriver
        .rightBumper()
        .whileTrue(
            new Intake(m_intake)
                .alongWith(
                    NoteVisualizer.speakerShoot()
                        .onlyIf(() -> m_arm.getState() == ArmStates.SHOOTING)));

    subsystemsDriver
        .x()
        .whileTrue(
            new SpeakerShoot(m_shooter)
                .alongWith(
                    m_arm.goToPosition(
                        SPEAKER_SCORING_POSITION, m_arm.changeState(ArmStates.SHOOTING))))
        .whileFalse(m_arm.goToPosition(IDLE_UNDER_STAGE, m_arm.changeState(ArmStates.IDLE)));

    subsystemsDriver
        .povLeft()
        .whileTrue(new UnderStageShoot(m_shooter))
        .whileFalse(m_arm.goToPosition(IDLE_UNDER_STAGE, m_arm.changeState(ArmStates.IDLE)));

    subsystemsDriver.povRight().whileTrue(new OverStageShoot(m_shooter));

    subsystemsDriver.b().whileTrue(new Outake(m_intake, m_shooter));

    subsystemsDriver
        .a()
        .onTrue(m_arm.goToPosition(AMP_POSITION, m_arm.changeState(ArmStates.STANDING)));
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

  public static Command pathfindAndAlignAmp() {
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
    NamedCommands.registerCommand("ShootSim", NoteVisualizer.speakerShoot());
    NamedCommands.registerCommand(
        "Intake",
        new ParallelCommandGroup(
            new IntakeWSensor(m_intake),
            m_arm.goToPosition(INTAKING_POSITION, m_arm.changeState(ArmStates.INTAKING))));
    NamedCommands.registerCommand(
        "Arm160",
        m_arm.goToPosition(SPEAKER_SCORING_POSITION, m_arm.changeState(ArmStates.SHOOTING)));
    NamedCommands.registerCommand(
        "Arm150", m_arm.goToPosition(150, m_arm.changeState(ArmStates.SHOOTING)));
    NamedCommands.registerCommand(
        "StarterShoot",
        new ParallelCommandGroup(
            new WaitCommand(1.3).andThen(NoteVisualizer.speakerShoot()),
            m_arm.goToPosition(SPEAKER_SCORING_POSITION, m_arm.changeState(ArmStates.SHOOTING))));
    NamedCommands.registerCommand("GetThatNote", new GoToNoteCommand(drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static ArmSubsystem getArmSubsystem() {
    return m_arm;
  }

  public static Drive getSwerveSubsystem() {
    return drive;
  }
}
