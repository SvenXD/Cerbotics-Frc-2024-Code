package frc.robot.Subsystems.Vision.Limelight.LimelightVision;

import static frc.robot.Constants.VisionConstants.*;

import frc.Util.LimelightHelpers;

public class LimelightVision {

  private String llName;

  public LimelightVision(int index) {

    llName = tagLimelightName + index;

    switch (index) {
      case 0:
        LimelightHelpers.setPipelineIndex(llName, index);
        break;

      case 1:
        LimelightHelpers.setPipelineIndex(llName, index);
        break;

      case 2:
        LimelightHelpers.setPipelineIndex(llName, index);
        break;

      case 3:
        LimelightHelpers.setPipelineIndex(llName, index);
        break;

      default:
        throw new IllegalArgumentException("Invalid index");
    }
  }

  public boolean hasTargets() {
    return LimelightHelpers.getTV(llName);
  }

  public double getTx() {
    return LimelightHelpers.getTX(llName);
  }

  public double getTy() {
    return LimelightHelpers.getTY(llName);
  }

  public int getNumOfTagsDetected() {
    return LimelightHelpers.getTargetCount(llName);
  }

  public String getLimelightname() {
    return llName;
  }
}
