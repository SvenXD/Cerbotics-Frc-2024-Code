package frc.robot.Subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import frc.Util.LimelightHelpers;

public class AprilTagIOLimelight implements AprilTagIO{

  private int currentPipeline = 0;

  public AprilTagIOLimelight(){
     
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs){
    inputs.distanceFromTarget = getDistanceToTarget();
    inputs.numOfTagsDetected = getNumofDetectedTargets();
    inputs.tX = getTx();
    inputs.tY = getTy();
    inputs.tV = getTv();
    inputs.currentPipeline = currentPipeline;
    inputs.test = test();
  }

    public double getDistanceToTarget(){
    double distance = 0.0;
    distance = LimelightHelpers.getTargetPose3d_CameraSpace(tagLimelightName).getZ();
    return distance;
  }

  public Pose3d test(){
    return LimelightHelpers.getBotPose3d_wpiRed(tagLimelightName);
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

  public void changePipeline(int pipeline){
    LimelightHelpers.setPipelineIndex(tagLimelightName, pipeline);
    currentPipeline = pipeline;
  }
  @Override
  public void changeSmartPipeline(){
    if(!getTv()){
      changePipeline(main_Pipeline);
    }
    else{
      changePipeline(upper_Pipeline);
    }
  } 
}
