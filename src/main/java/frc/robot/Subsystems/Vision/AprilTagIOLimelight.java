package frc.robot.Subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;
import frc.Util.LimelightHelpers;

public class AprilTagIOLimelight implements AprilTagIO{


  public AprilTagIOLimelight(){
     
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs){
    inputs.distanceFromTarget = getDistanceToTarget();
    inputs.numOfTagsDetected = getNumofDetectedTargets();
    inputs.tX = getTx();
    inputs.tY = getTy();
    inputs.tV = getTv();
  }

    public double getDistanceToTarget(){
    double distance = 0.0;
    distance = LimelightHelpers.getTargetPose3d_CameraSpace(tagLimelightName).getZ();
    return distance;
  }
    
    public double getNumofDetectedTargets(){
    return LimelightHelpers.getTargetCount(tagLimelightName);
  }

    public double getTx(){
      return LimelightHelpers.getTX(tagLimelightName);
  }

    public double getTy(){
      return LimelightHelpers.getTY(tagLimelightName);
  }
  
  public boolean getTv(){
      return LimelightHelpers.getTV(tagLimelightName);
  }
}
