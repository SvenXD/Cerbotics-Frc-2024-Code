
package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Drive;

public class PhotonAprilTagVision extends SubsystemBase {
  PhotonSim[] cameras;
  Drive m_drive;

  public PhotonAprilTagVision( Drive m_drive, PhotonSim... cameras) {
    this.cameras = cameras;
    this.m_drive = m_drive;
  }

  @Override
  public void periodic() {
    for(int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++){
      var visionEst = cameras[instanceIndex].getEstimatedGlobalPose();
      List<Pose3d> tagPose3ds = new ArrayList<>();
      PhotonPipelineResult unprocessedResult = cameras[instanceIndex].getLatestResult();
      final int index = instanceIndex;
      cameras[instanceIndex].ActivateSimParameters(m_drive.getPose() , instanceIndex);

       for (int id : unprocessedResult.getMultiTagResult().fiducialIDsUsed) {
          tagPose3ds.add(VisionConstants.kTagLayout.getTagPose(id).get());
        }      

      visionEst.ifPresent(
              est -> {
                  var estPose = est.estimatedPose.toPose2d();
                  // Change our trust in the measurement based on the tags we can see
                  var estStdDevs = cameras[index].getEstimationStdDevs(estPose);

                  m_drive.addVisionMeasurement(
                          est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
              });
              
  
        Logger.recordOutput("Photon/Tags Used " + instanceIndex, tagPose3ds.size());

    }
  }

    public void setDynamicVisionStdDevs(){
  
  }
}
