package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagLocalizationIO {

  @AutoLog
  public static class AprilTagLocalizationIOInputs {
    // Pose Estimate 1
    public Pose3d pose0;
    public double ambiguity0;
    public double averageTagDistance0;
    public int[] tagIDs0;

    // Pose Estimate 2
    public Pose3d pose1;
    public double ambiguity1;
    public double averageTagDistance1;
    public int[] tagIDs1;

    public double timestamp;
  }

  public default void updateInputs(AprilTagLocalizationIOInputs inputs) {}

  public record AprilTagPoseEstimate(
      Pose3d pose0,
      double ambiguity0,
      double averageTagDistance0,
      Pose3d pose1,
      double ambiguity1,
      double averageTagDistance1,
      double timestamp,
      int[] tagIDs) {}
}
