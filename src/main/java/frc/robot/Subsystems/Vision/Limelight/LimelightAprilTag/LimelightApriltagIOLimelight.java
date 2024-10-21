package frc.robot.Subsystems.Vision.Limelight.LimelightAprilTag;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedString;
import frc.Util.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class LimelightApriltagIOLimelight implements LimelightAprilTagIO {

  private String limelightName = "Placeholder";
  public int indexPlaceholder = 0;
  private final StringSubscriber jsonDumpSub =
      NetworkTableInstance.getDefault()
          .getTable(limelightName)
          .getStringTopic("json")
          .subscribe("", PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

  public LimelightApriltagIOLimelight(int index) {

    switch (index) {
      case 0:
        limelightName = VisionConstants.tagLimelightName + index;
        LimelightHelpers.setPipelineIndex(limelightName, index);
        break;

      case 1:
        limelightName = VisionConstants.tagLimelightName + index;
        LimelightHelpers.setPipelineIndex(VisionConstants.tagLimelightName + index, index);
        break;

      case 2:
        limelightName = VisionConstants.tagLimelightName + index;
        LimelightHelpers.setPipelineIndex(VisionConstants.tagLimelightName + index, index);
        break;

      case 3:
        limelightName = VisionConstants.tagLimelightName + index;
        LimelightHelpers.setPipelineIndex(VisionConstants.tagLimelightName + index, index);
        break;

      default:
        throw new IllegalArgumentException("Invalid index");
    }
    indexPlaceholder = index;
  }

  public String getLimelightname() {
    return limelightName;
  }

  public int getLimelightIndex() {
    return indexPlaceholder;
  }

  @Override
  public void updateInputs(LimelightAprilTagInputs inputs) {
    TimestampedString[] queue = jsonDumpSub.readQueue();

    for (TimestampedString timestampedString : queue) {
      double timestamp = timestampedString.timestamp / 1000000.0;
      LimelightHelpers.LimelightResults results =
          LimelightHelpers.getLatestResults(timestampedString.value);
    }
  }
}
