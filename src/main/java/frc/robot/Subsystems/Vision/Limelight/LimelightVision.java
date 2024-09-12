package frc.robot.Subsystems.Vision.Limelight;

import frc.Util.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class LimelightVision {

  private String limelightName = "Placeholder";

  public LimelightVision(int index) {

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
  }

  public boolean hasTargets() {
    return LimelightHelpers.getTV(limelightName);
  }

  public double getTx() {
    return LimelightHelpers.getTX(limelightName);
  }

  public double getTy() {
    return LimelightHelpers.getTY(limelightName);
  }

  public int getNumOfTagsDetected() {
    return LimelightHelpers.getTargetCount(limelightName);
  }

  public String getLimelightname() {
    return limelightName;
  }
}
