package frc.robot.Subsystems.Vision;

import static frc.robot.Constants.VisionConstants.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;

public class ApriltagIOPhotonvision implements AprilTagIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;

    public ApriltagIOPhotonvision(int index){
     switch(index){
        case(1):
          camera = new PhotonCamera(photonCam1);
            photonEstimator = new PhotonPoseEstimator(
                kTagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                camera,
                kRobotToCam1);
            break;
        case(2):
          camera = new PhotonCamera(photonCam2);
            photonEstimator = new PhotonPoseEstimator(
                kTagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                camera,
                kRobotToCam2);
            break;
        case(3):
          camera = new PhotonCamera(photonCam3);
            photonEstimator = new PhotonPoseEstimator(
                kTagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                camera,
                kRobotToCam3);
            break;
        case(4):
          camera = new PhotonCamera(photonCam4);
            photonEstimator = new PhotonPoseEstimator(
                kTagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                camera,
                kRobotToCam4);
            break;                                    
            default:
            throw new RuntimeException("Invalid module index");    
        }
    }

  public PhotonPipelineResult getLatestResult() {
      return camera.getLatestResult();
    }                                                           
        
/**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }            
}
