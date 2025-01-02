// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.CTRE.swerve.SwerveModule.DriveRequestType;
import frc.Util.CTRE.swerve.SwerveRequest;
import frc.Util.CTRE.swerve.SwerveRequest.ForwardReference;
import frc.Util.LocalADStarAK;
import frc.robot.Commands.SwerveCommands.FieldCentricDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Swerve.CTRESwerve.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Swerve.CTRESwerve.CommandSwerveDrivetrain.driveMethod;
import frc.robot.Subsystems.Swerve.CTRESwerve.TunerConstants;

public class RobotContainer {

  public static final CommandXboxController chassisDriver = new CommandXboxController(0);
  public static final CommandXboxController subsystemsDriver = new CommandXboxController(1);

  /*private static LoggedDashboardChooser<AutoCommand> autoChooser;
  private static final SendableChooser<Pose2d> poseChooser = new SendableChooser<>();

  public static Field2d autoPreviewField = new Field2d();

  public static ArmIO armIO = new ArmIOKraken();
  public static ArmSubsystem m_arm = new ArmSubsystem(armIO);

  public static IntakeIO intakeIO = new IntakeIOKraken();
  public static IntakeSubsystem m_intake = new IntakeSubsystem(intakeIO);

  public static ShooterIO shooterIO = new ShooterIOTalon();
  public static ShooterSubsystem m_shooter = new ShooterSubsystem(shooterIO);

  // public static ClimberIO climberIO = new ClimberIOKraken();
  // public static ClimberSubsystem m_climber = new ClimberSubsystem(climberIO);*/

  private static final CommandSwerveDrivetrain m_drive = TunerConstants.DriveTrain;

  SwerveRequest.FieldCentricFacingAngle m_head =
      new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity);

  /*private static final VisionSubsystem m_vision =
        new VisionSubsystem(m_drive, Constants.VisionConstants.tagLimelightName);
  */
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
    // registerNamedCommands();
    /** Visualisation of the current auto selected * */
    /*autoChooser = new LoggedDashboardChooser<>("Auto Mode");

    autoChooser.onChange(
        auto -> {
          autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
        });

    /* Auto options */
    /*autoChooser.addDefaultOption("None", new NoneAuto());
    autoChooser.addOption("4 Note Auto", new MoveTest());

    /* Aligning options */
    /*poseChooser.setDefaultOption("Speaker", FieldConstants.redSpeakerPose);
        poseChooser.addOption("Amp", FieldConstants.redAmpPose);

        PathPlannerLogging.setLogActivePathCallback(
            (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
        PathPlannerLogging.setLogTargetPoseCallback(
            pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

        SmartDashboard.putData("Auto Preview", autoPreviewField);
        SmartDashboard.putData("Pose Chooser", poseChooser);
    */
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
                () -> chassisDriver.getLeftY(),
                () -> chassisDriver.getLeftX(),
                () -> chassisDriver.getRightX())
            .onlyIf(() -> m_drive.driveState == driveMethod.JOYSTICK));

    chassisDriver.a().onTrue(m_drive.runOnce(() -> m_drive.seedFieldRelative()));
  }

  /*chassisDriver
      .rightBumper()
      .whileTrue(m_arm.goToPosition(173).alongWith(new IntakeWithSensor(m_intake)))
      .whileFalse(m_arm.goToPosition(172));

  chassisDriver
      .rightBumper()
      .and(() -> m_intake.isNoteInside())
      .onTrue(controllerRumbleCommand().withTimeout(1));

  chassisDriver
      .b()
      .whileTrue(m_shooter.setRpms(-10, -10).alongWith(m_intake.setall(0.5, 0.5)))
      .whileFalse(m_intake.setall(0, 0).alongWith(m_shooter.stop()));

  chassisDriver
      .y()
      .onTrue(
          pathfindAndAlignAmp()
              .until(
                  chassisDriver
                      .axisGreaterThan(1, 0.1)
                      .or(chassisDriver.axisGreaterThan(0, 0.1))
                      .or(chassisDriver.axisGreaterThan(4, 0.1))
                      .onTrue(
                          m_vision
                              .setVisionTrue()
                              .alongWith(
                                  new InstantCommand(
                                      () -> m_drive.changeDriveMethod(driveMethod.JOYSTICK))))));

  /* Driver 2 */
  /*subsystemsDriver
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
        .rightBumper()
        .whileTrue(m_intake.setUpperVoltage(-1))
        .whileFalse(
            m_intake
                .setUpperVoltage(0)
                .alongWith(m_arm.goToPosition(172))
                .alongWith(m_shooter.stop()));
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

  public static Command pathfindAndAlignAmp() {
    return Commands.sequence(
        m_vision.changeVision(),
        Commands.either(
                m_drive
                    .goToPose(FieldConstants.redAmpPose)
                    .until(
                        () ->
                            m_drive
                                    .getState()
                                    .Pose
                                    .getTranslation()
                                    .getDistance(FieldConstants.redAmpPose.getTranslation())
                                <= 3),
                m_drive
                    .goToPose(FieldConstants.blueAmpPose)
                    .until(
                        () ->
                            m_drive
                                    .getState()
                                    .Pose
                                    .getTranslation()
                                    .getDistance(FieldConstants.blueAmpPose.getTranslation())
                                <= 3),
                Robot::isRedAlliance)
            .andThen(
                new DeferredCommand(
                    () -> {
                      Pose2d currentPose = m_drive.getState().Pose;
                      ChassisSpeeds currentSpeeds =
                          ChassisSpeeds.fromRobotRelativeSpeeds(
                              m_drive.getCurrentFieldChassisSpeeds(), currentPose.getRotation());

                      Rotation2d heading =
                          new Rotation2d(
                              currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

                      Pose2d targetPose =
                          Robot.isRedAlliance()
                              ? FieldConstants.redAmpPose
                              : FieldConstants.blueAmpPose;
                      var bezierPoints =
                          PathPlannerPath.bezierFromPoses(
                              new Pose2d(currentPose.getTranslation(), heading), targetPose);
                      PathPlannerPath path =
                          new PathPlannerPath(
                              bezierPoints,
                              new PathConstraints(
                                  2.0,
                                  3.0,
                                  Units.degreesToRadians(360),
                                  Units.degreesToRadians(360)),
                              new GoalEndState(0.0, targetPose.getRotation(), true));
                      path.preventFlipping = true;

                      return AutoBuilder.followPath(path);
                    },
                    Set.of(m_drive)),
                new AmpRoutine(m_arm, m_shooter, m_intake).andThen(m_vision.changeVision())));
  }

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
  }*/

  public Command getAutonomousCommand() {
    Command ass = new PathPlannerAuto("New New Auto");
    return ass;
  }

  public static CommandSwerveDrivetrain getDrive() {
    return m_drive;
  }

  /*public static ArmSubsystem getArm() {
    return m_arm;
  }

  public static ShooterSubsystem getShooter() {
    return m_shooter;
  }

  public static IntakeSubsystem getIntake() {
    return m_intake;
  }*/
}
