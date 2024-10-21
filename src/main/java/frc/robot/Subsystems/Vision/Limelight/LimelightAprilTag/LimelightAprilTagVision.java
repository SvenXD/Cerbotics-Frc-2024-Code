package frc.robot.Subsystems.Vision.Limelight.LimelightAprilTag;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.robot.Subsystems.Swerve.Drive;
import org.littletonrobotics.junction.Logger;

public class LimelightAprilTagVision extends SubsystemBase {
  private final LimelightAprilTagIO io;
  private final LimelightAprilTagInputsAutoLogged inputs = new LimelightAprilTagInputsAutoLogged();

  private final Drive m_drive;
  private final Field2d m_field = new Field2d();

  public LimelightAprilTagVision(Drive m_drive, LimelightAprilTagIO io) {
    this.m_drive = m_drive;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LimelightVision", inputs);
    odometryWithVision();

    /*double xyStdDev =
        Constants.VisionConstants.xyStdDevCoefficient * Math.pow(avgDistance, 2) / tagPoses.size();
    double thetaStdDev =
        Constants.VisionConstants.thetaStdDevCoefficient
            * Math.pow(avgDistance, 2)
            / tagPoses.size();*/
  }

  public void odometryWithVision() {

    if (inputs.hasTarget) {
      boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(
          "limelight[index].getLimelightname()", m_drive.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              "limelight[index].getLimelightname()");
      if (Math.abs(m_drive.getangle())
          > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision
      // updates
      {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
        m_drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      }
    }

    m_field.getObject("limelight[index].getLimelightname()").setPose(m_drive.getPose());
  }
}
