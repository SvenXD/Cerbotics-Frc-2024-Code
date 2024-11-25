package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.CTRESwerve.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain m_drive;
  private final Field2d m_field = new Field2d();
  private String limelightNames;
  private double averageTagDistance = 0.0;

  public VisionSubsystem(CommandSwerveDrivetrain m_drive, String limelightNames) {
    this.m_drive = m_drive;
    this.limelightNames = limelightNames;
  }

  @Override
  public void periodic() {
    odometryWithVision(limelightNames, 0, 0);

    int[] tagIDs =
        new int[LimelightHelpers.getLatestResults(limelightNames).targets_Fiducials.length];

    for (int i = 0;
        i < LimelightHelpers.getLatestResults(limelightNames).targets_Fiducials.length;
        i++) {
      tagIDs[i] =
          (int)
              Math.round(
                  LimelightHelpers.getLatestResults(limelightNames)
                      .targets_Fiducials[i]
                      .fiducialID);
      averageTagDistance +=
          LimelightHelpers.getLatestResults(limelightNames)
              .targets_Fiducials[i]
              .getTargetPose_CameraSpace()
              .getTranslation()
              .getNorm();
    }
    averageTagDistance /= tagIDs.length;

    double xyStdDev =
        Constants.VisionConstants.xyStdDevCoefficient
                    * Math.pow(averageTagDistance, 2)
                    / tagIDs.length
                == 0
            ? 0.1
            : tagIDs.length;
    double thetaStdDev =
        Constants.VisionConstants.thetaStdDevCoefficient
                    * Math.pow(averageTagDistance, 2)
                    / tagIDs.length
                == 0
            ? 0.1
            : tagIDs.length;

    Logger.recordOutput("Vision/Tag Distance", averageTagDistance);
    SmartDashboard.putBoolean("tesrtvision", LimelightHelpers.getTV(limelightNames));

    SmartDashboard.putData(m_field);
  }

  public void odometryWithVision(String limelightName, double xySTD, double thetaSTD) {

    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation(
        limelightName, m_drive.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    /*if (Math.abs(m_drive.getState().Pose.getRotation().getDegrees())
        > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision
    // updates
    {
      doRejectUpdate = true;
    }
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    /*if (mt2.pose.getX() < -Constants.FieldConstants.fieldBorderMargin
        || mt2.pose.getX()
            > Constants.FieldConstants.fieldSize.getX()
                + Constants.FieldConstants.fieldBorderMargin
        || mt2.pose.getY() < -Constants.FieldConstants.fieldBorderMargin
        || mt2.pose.getY()
            > Constants.FieldConstants.fieldSize.getY()
                + Constants.FieldConstants.fieldBorderMargin) {
      doRejectUpdate = false;
    }*/
    if (!doRejectUpdate) {
      m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.1));
      m_drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }
    m_field.getObject(limelightName).setPose(mt2.pose);

    SmartDashboard.putBoolean(limelightName, LimelightHelpers.getTV(limelightName));
  }
}
