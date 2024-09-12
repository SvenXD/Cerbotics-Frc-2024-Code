package frc.robot.Subsystems.Vision.Limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.robot.Subsystems.Swerve.Drive;

public class LimelightAprilTagVision extends SubsystemBase {
  LimelightVision[] limelight;
  Drive m_drive;
  Field2d m_field = new Field2d();

  public LimelightAprilTagVision(Drive m_drive, LimelightVision... limelight) {
    this.limelight = limelight;
    this.m_drive = m_drive;
  }

  @Override
  public void periodic() {
    for (int instanceIndex = 0; instanceIndex < limelight.length; instanceIndex++) {
      odometryWithVision(instanceIndex);
    }
  }

  public void odometryWithVision(int index) {
    if (limelight[index].hasTargets()) {
      boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(
          limelight[index].getLimelightname(), m_drive.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight[index].getLimelightname());
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

    m_field.getObject(limelight[index].getLimelightname()).setPose(m_drive.getPose());
  }
}
