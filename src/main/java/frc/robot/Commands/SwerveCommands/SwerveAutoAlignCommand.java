package frc.robot.Commands.SwerveCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Util.CTRE.swerve.SwerveModule.DriveRequestType;
import frc.Util.CTRE.swerve.SwerveRequest;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SwerveAutoAlignCommand extends Command {
  private final Pose2d redPose;
  private final Pose2d bluePose;
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private Pose2d targetPose;

  public SwerveAutoAlignCommand(Pose2d redPose, Pose2d bluePose) {

    this.redPose = redPose;
    this.bluePose = bluePose;

    this.xController =
        new ProfiledPIDController(0.2, 0, 0, new TrapezoidProfile.Constraints(2.0, 2.0));
    this.yController =
        new ProfiledPIDController(0.2, 0, 0, new TrapezoidProfile.Constraints(2.0, 2.0));
    this.thetaController =
        new ProfiledPIDController(
            0.0,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(180), Units.degreesToRadians(180)));
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(RobotContainer.getDrive());
  }

  @Override
  public void initialize() {
    if (Robot.isRedAlliance()) {
      targetPose = redPose;
    } else {
      targetPose = bluePose;
    }

    Pose2d currentPose = RobotContainer.getDrive().getState().Pose;
    ChassisSpeeds currentSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            RobotContainer.getDrive().getCurrentFieldChassisSpeeds(), currentPose.getRotation());

    xController.reset(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
    yController.reset(currentPose.getY(), currentSpeeds.vyMetersPerSecond);
    thetaController.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    Pose2d currentPose = RobotContainer.getDrive().getState().Pose;

    double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotFeedback =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    double xFF = xController.getSetpoint().velocity;
    double yFF = yController.getSetpoint().velocity;
    double rotFF = thetaController.getSetpoint().velocity;

    double xVel = xFF + xFeedback;
    double yVel = yFF + yFeedback;
    double rotVel = rotFF + rotFeedback;

    if (Math.abs(currentPose.getX() - targetPose.getX()) < 0.025) {
      xVel = 0;
    }
    if (Math.abs(currentPose.getY() - targetPose.getY()) < 0.025) {
      yVel = 0;
    }
    if (Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 30) {
      rotVel = 0;
    }

    RobotContainer.getDrive()
        .setControl(drive.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(rotVel));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    Pose2d currentPose = RobotContainer.getDrive().getState().Pose;
    return Math.abs(currentPose.getX() - targetPose.getX()) < 0.025
        && Math.abs(currentPose.getY() - targetPose.getY()) < 0.025;
  }
}
