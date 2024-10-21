package frc.robot.Subsystems.Vision.Limelight.LimelightAprilTag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightAprilTagIO {
  @AutoLog
  class LimelightAprilTagInputs {
    public boolean hasTarget = false;
    public double tx = 0.0;
    public double ty = 0.0;
    public double timestamp = 0.0;
    public double fps = 0.0;
    public double lastFPSTimestamp = 0.0;
    public double distanceFromTarget = 0.0;
    public Pose2d currentTagPositions = new Pose2d(0, 0, new Rotation2d());

    public String limeLightName = " ";
  }

  void updateInputs(LimelightAprilTagInputs inputs);
}
