package frc.robot.Subsystems.Vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Robot;

public class AprilTagIOSim implements AprilTagIO {
  private final PhotonPoseEstimator photonEstimator;
  private final VisionSystemSim visionSim = new VisionSystemSim("main");
  private final TargetModel targetModel = new TargetModel(0.5, 0.25);
  private final PhotonCamera camera = new PhotonCamera("cameraName");     //This camera is just for the simulation
  private final Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5); //Meters
  private final Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0); //Degrees
  private final Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
  private final PhotonCameraSim cameraSim; 
  private final Pose3d targetPose;
  private final VisionTargetSim visionTarget; //Custom April tag
  private final SimCameraProperties cameraProp; 
  private double lastEstTimestamp = 0;

  public AprilTagIOSim(){
    cameraProp = new SimCameraProperties();

    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);
   
    cameraSim = new PhotonCameraSim(camera, cameraProp);

    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);

      cameraSim.enableDrawWireframe(true);
      targetPose = new Pose3d(16, 4, 0.4, new Rotation3d(0, 0, Math.PI));
      visionTarget = new VisionTargetSim(targetPose, targetModel);

      visionSim.addAprilTags(VisionConstants.kTagLayout);
      visionSim.addCamera(cameraSim, robotToCamera);

            photonEstimator = new PhotonPoseEstimator(
                VisionConstants.kTagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                camera,
                robotToCamera);

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
        if (Robot.isSimulation()) {
            visionEst.ifPresentOrElse(
                    est ->
                            getSimDebugField()
                                    .getObject("VisionEstimation")
                                    .setPose(est.estimatedPose.toPose2d()),
                    () -> {
                        if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                    });
        }
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
        
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
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
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }  

    public double getDistanceToTarget(){
        return PhotonUtils.calculateDistanceToTargetMeters(
            0.5,
            1.32,
            Math.toRadians(-15),
            Units.degreesToRadians(antiNull()));
    }

    public double antiNull(){
        double val = 1;
        if(getLatestResult().getBestTarget() == null){
            val = 1;
        }
        else{
            val =getLatestResult().getBestTarget().getArea();
        }
        return val;
    }

    @Override
    public void measurements(Drive m_drive){
        var visionEst = getEstimatedGlobalPose();
        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = getEstimationStdDevs(estPose);

                    m_drive.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                            });
    }


    @Override
    public void ActivateSimParameters(Pose2d robotPoseMeters){
        Logger.recordOutput("Vision/Tags", targetPose);
        Logger.recordOutput("Vision/Cameras", new Pose3d(0.1,0,0.5,robotToCameraRot));
        visionSim.update(robotPoseMeters);
    }
    
    @Override
    public void updateInputs(AprilTagIOInputs inputs){
      inputs.tV = getLatestResult().hasTargets();  
      inputs.distanceFromTarget = Math.abs(getDistanceToTarget());
    }

}
