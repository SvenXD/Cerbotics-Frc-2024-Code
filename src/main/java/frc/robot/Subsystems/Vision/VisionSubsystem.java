package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.CTRESwerve.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain m_drive;
  private final Field2d m_field = new Field2d();
  private String limelightNames;
  private double averageTagDistance = 0.0;
  private static LimelightHelpers.PoseEstimate mt2;

  public static enum reasonForRejectUpdate {
    outOfTheField,
    noTag,
    overThreshold;
  }

  private reasonForRejectUpdate systemStates = reasonForRejectUpdate.noTag;

  public VisionSubsystem(CommandSwerveDrivetrain m_drive, String limelightNames) {
    this.m_drive = m_drive;
    this.limelightNames = limelightNames;
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames);
  }

  @Override
  public void periodic() {
    int tagsDetected = LimelightHelpers.getTargetCount(limelightNames);

    averageTagDistance = mt2.avgTagDist;

    double xyStdDev =
        Constants.VisionConstants.xyStdDevCoefficient
            * Math.pow(averageTagDistance, 2)
            / (tagsDetected == 0 ? 100 : tagsDetected);
    double thetaStdDev =
        Constants.VisionConstants.thetaStdDevCoefficient
            * Math.pow(averageTagDistance, 2)
            / (tagsDetected == 0 ? 100 : tagsDetected);

    odometryWithVision(VisionConstants.tagLimelightName, xyStdDev, thetaStdDev);

    SmartDashboard.putNumber("Distance from tag", averageTagDistance);
    SmartDashboard.putNumber("XY STD", xyStdDev);
    SmartDashboard.putNumber("Theta STD", thetaStdDev);
    SmartDashboard.putString("Reason for rejected vision", systemStates.toString());
    SmartDashboard.putData(m_field);
  }

  public void odometryWithVision(String limelightName, double xySTD, double thetaSTD) {
    LimelightHelpers.PoseEstimate oldMt = mt2;
    boolean doRejectUpdate = false;
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    double timeDiff = (mt2.timestampSeconds - oldMt.timestampSeconds) * 20;

    if (mt2.pose.getX() < -Constants.FieldConstants.fieldBorderMargin
        || mt2.pose.getX()
            > Constants.FieldConstants.fieldSize.getX() + Constants.FieldConstants.fieldBorderMargin
        || mt2.pose.getY() < -Constants.FieldConstants.fieldBorderMargin
        || mt2.pose.getY()
            > Constants.FieldConstants.fieldSize.getY()
                + Constants.FieldConstants.fieldBorderMargin) {
      doRejectUpdate = true;
      systemStates = reasonForRejectUpdate.outOfTheField;
    }

    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
      systemStates = reasonForRejectUpdate.noTag;
    }

    if (Math.abs(oldMt.pose.getX() - mt2.pose.getX()) > timeDiff
        || Math.abs(oldMt.pose.getY() - mt2.pose.getY()) > timeDiff) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(xySTD, xySTD, thetaSTD));
      m_drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }
    m_field.getObject(limelightName).setPose(m_drive.getState().Pose);

    SmartDashboard.putNumber("Difference of times", timeDiff);
  }
}
