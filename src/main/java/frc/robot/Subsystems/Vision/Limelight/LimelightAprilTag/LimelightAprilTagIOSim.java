package frc.robot.Subsystems.Vision.Limelight.LimelightAprilTag;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class LimelightAprilTagIOSim implements LimelightAprilTagIO {
  private double x = 0;
  private double y = 0;
  private double z = 0;
  private final List<Translation3d> aprilTagPositions =
      List.of(
          new Translation3d(15.083472, 0.245472, 1.355852),
          new Translation3d(16.189634, 0.883466, 1.355852),
          new Translation3d(16.584442, 4.981718, 1.451202),
          new Translation3d(16.584442, 5.548452, 1.451202),
          new Translation3d(14.699358, 8.2042, 1.355852),
          new Translation3d(1.841, 8.2042, 1.355852),
          new Translation3d(-0.0381, 5.548452, 1.451202),
          new Translation3d(-0.0381, 4.981718, 1.451202),
          new Translation3d(0.356108, 0.883466, 1.355852),
          new Translation3d(1.46227, 0.245472, 1.355852),
          new Translation3d(11.903426, 3.719274, 1.3208),
          new Translation3d(11.903426, 45.1058, 1.3208),
          new Translation3d(11.219644, 41.055448, 1.3208),
          new Translation3d(5.320792, 4.104524, 1.3208),
          new Translation3d(5.320792, 45.1058, 1.3208),
          new Translation3d(4.642602, 40.20458, 1.3208));

  @Override
  public void updateInputs(LimelightAprilTagInputs inputs) {
    Pose3d robotPose = new Pose3d(RobotContainer.getSwerveSubsystem().getPose());
    Pose3d llPose =
        robotPose.transformBy(
            new Transform3d(
                Constants.VisionConstants.noteCam.getTranslation(),
                Constants.VisionConstants.noteCam.getRotation()));

    Translation3d target = null;
    for (Translation3d pos : aprilTagPositions) {
      Rotation2d angleToObj =
          pos.toTranslation2d().minus(llPose.getTranslation().toTranslation2d()).getAngle();
      Rotation2d diff = llPose.getRotation().toRotation2d().minus(angleToObj);

      if (Math.abs(diff.getDegrees()) <= (63.3 / 2.0)) {
        double distance = llPose.getTranslation().getDistance(pos);
        if (distance <= 6.0 && target == null) {
          target = pos;

        } else if (target != null && distance < llPose.getTranslation().getDistance(target)) {
          target = pos;
        }
        if (target == null) {

          x = 0;
          y = 0;
          z = 0;
        } else {
          x = target.getX();
          y = target.getY();
          z = target.getZ();
        }
        Logger.recordOutput("Vision/AprilTagTest", new Pose3d(x, y, z, new Rotation3d()));
      }
    }

    if (target != null) {
      inputs.hasTarget = true;
      inputs.timestamp = Timer.getFPGATimestamp();

      Rotation2d angleToObj =
          target.toTranslation2d().minus(llPose.getTranslation().toTranslation2d()).getAngle();
      Rotation2d diff = llPose.getRotation().toRotation2d().minus(angleToObj);
      inputs.tx = diff.getDegrees();

      double h = llPose.getTranslation().getDistance(target);
      double o = llPose.getZ() - target.getZ();
      inputs.ty = -Units.radiansToDegrees(Math.asin(o / h) - llPose.getRotation().getY());

      double distance =
          Math.sqrt(
              Math.pow(llPose.getX() - target.getX(), 2)
                  + Math.pow(llPose.getY() - target.getY(), 2));

      inputs.distanceFromTarget = distance;
    } else {
      inputs.hasTarget = false;
      inputs.distanceFromTarget = 0;
    }

    inputs.fps = 30.0;
    inputs.lastFPSTimestamp = Timer.getFPGATimestamp();
    inputs.currentTagPositions = new Pose2d(x, y, new Rotation2d());
  }
}
