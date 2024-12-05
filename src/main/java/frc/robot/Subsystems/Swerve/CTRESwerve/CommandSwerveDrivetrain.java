package frc.robot.Subsystems.Swerve.CTRESwerve;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Util.CTRE.swerve.SwerveDrivetrain;
import frc.Util.CTRE.swerve.SwerveDrivetrainConstants;
import frc.Util.CTRE.swerve.SwerveModule.DriveRequestType;
import frc.Util.CTRE.swerve.SwerveModuleConstants;
import frc.Util.CTRE.swerve.SwerveRequest;
import frc.Util.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final PathConstraints constraints = new PathConstraints(1.0, 1.0, 0, 0);
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  // Check how to fix field centric
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  Rotation2d velocityOffset = new Rotation2d();

  private final SwerveRequest.FieldCentric requestForAlign =
      new SwerveRequest.FieldCentric()
          .withDeadband(DriveConstants.MaxLinearSpeed * 0.1)
          .withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  private SysIdRoutine SysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(TranslationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine SysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(RotationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine SysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(7),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(SteerCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine RoutineToApply = SysIdRoutineRotation;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    configurePathPlanner();
    SignalLogger.start();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds) ->
            this.setControl(
                AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(
            new PIDConstants(DriveConstants.traslationP, DriveConstants.traslationD),
            new PIDConstants(DriveConstants.rotationP, DriveConstants.rotationD),
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().orElse(Alliance.Blue)
                == Alliance
                    .Red, // Assume the path needs to be flipped for Red vs Blue, this is normally
        // the case
        this); // Subsystem for requirements
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command goToPose(Pose2d pose) {
    return AutoBuilder.pathfindToPose(pose, constraints, 0.0, 0.0);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public ChassisSpeeds getCurrentFieldChassisSpeeds() {
    var state = getState();
    var robotAngle = state.Pose.getRotation();
    var chassisSpeeds = state.speeds;
    var fieldSpeeds =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(robotAngle);
    return new ChassisSpeeds(
        fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return RoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return RoutineToApply.dynamic(direction);
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getCurrentRobotChassisSpeeds(), getRotation());
  }

  public Rotation2d getRotation() {
    return getState().Pose.getRotation();
  }

  public Rotation2d getVelocityOffset() {
    return velocityOffset;
  }

  public void setVelocityOffset(Rotation2d angle) {
    velocityOffset = angle;
  }

  public Rotation2d getAngleToTarget() {
    return Rotation2d.fromDegrees(-LimelightHelpers.getTX("null"));
  }

  public Rotation2d getTY() {
    return Rotation2d.fromDegrees(LimelightHelpers.getTY("null"));
  }

  public double getMeasurementTimestamp() {
    return Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Pipeline("null");
  }

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
    /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
    /* This allows us to correct the perspective in case the robot code restarts mid-match */
    /* Otherwise, only check and apply the operator perspective if the DS is disabled */
    /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/

    if (!hasAppliedOperatorPerspective || DriverStation.isTeleop()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RedAlliancePerspectiveRotation
                        : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
    }
    SmartDashboard.putNumber("TestVel", this.getState().speeds.vxMetersPerSecond);
  }

  public Command driveToIntakeRing() {
    PIDController rotationController = new PIDController(4, 0.0, 0.0);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    PIDController speedController = new PIDController(0.07, 0.0, 0.0);

    return Commands.sequence(
            Commands.either(
                run(() -> {
                      if (!LimelightHelpers.getTV("null")
                          || RobotContainer.getIntake().isNoteInside()) {
                        this.setControl(requestForAlign);
                        return;
                      }

                      var pose = this.getPoseAtTime(getMeasurementTimestamp());

                      if (pose.isEmpty()) {
                        this.setControl(requestForAlign);
                        return;
                      }

                      Rotation2d angleToNoteField =
                          pose.get()
                              .getRotation()
                              .plus(Rotation2d.fromDegrees(180))
                              .plus(getAngleToTarget());

                      Pose2d currentPose = this.getState().Pose;

                      double angVel =
                          rotationController.calculate(
                              currentPose
                                  .getRotation()
                                  .plus(Rotation2d.fromDegrees(180))
                                  .getRadians(),
                              angleToNoteField.getRadians());
                      this.setControl(
                          requestForAlign
                              .withVelocityX(0)
                              .withVelocityY(0)
                              .withRotationalRate(angVel));
                    })
                    .withTimeout(0.25), // Robot is rotating, rotate towards note for a bit
                Commands.none(),
                () ->
                    Math.abs(this.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond)
                        >= Units.degreesToRadians(180)),
            run(() -> {
                  if (!LimelightHelpers.getTV("null")
                      || RobotContainer.getIntake().isNoteInside()) {
                    this.setControl(requestForAlign);
                    return;
                  }

                  var pose = this.getPoseAtTime(getMeasurementTimestamp());

                  if (pose.isEmpty()) {
                    this.setControl(requestForAlign);
                    return;
                  }

                  Rotation2d angleToNoteField =
                      pose.get()
                          .getRotation()
                          .plus(Rotation2d.fromDegrees(180))
                          .plus(getAngleToTarget());
                  Rotation2d ty = getTY();

                  Pose2d currentPose = this.getState().Pose;

                  double angVel =
                      rotationController.calculate(
                          currentPose.getRotation().plus(Rotation2d.fromDegrees(180)).getRadians(),
                          angleToNoteField.getRadians());
                  double speed =
                      speedController.calculate(ty.plus(Rotation2d.fromDegrees(30)).getDegrees());

                  Translation2d fieldSpeeds =
                      new Translation2d(speed, angleToNoteField.plus(Rotation2d.fromDegrees(180)));

                  this.setControl(
                      requestForAlign
                          .withVelocityX(fieldSpeeds.getX())
                          .withVelocityY(fieldSpeeds.getY())
                          .withRotationalRate(angVel));
                })
                .onlyWhile(() -> LimelightHelpers.getTV("null")))
        .repeatedly();
  }
}
