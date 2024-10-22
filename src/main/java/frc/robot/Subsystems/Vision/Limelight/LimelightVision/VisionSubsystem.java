/*// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision.Limelight.LimelightVision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Drive;


public class VisionSubsystem extends SubsystemBase {

  private final Drive m_drive;
  private final LimelightVision[] m_vision;

  private final Field2d m_field = new Field2d();

  private static double xyStdDev = 0;
  private static double thetaStdDev = 0;

  public VisionSubsystem(Drive m_drive, LimelightVision ... m_vision) {
    this.m_drive = m_drive;
    this.m_vision = m_vision;
  }

  @Override
  public void periodic() {
    odometryWithVision();

    double xyStdDev =
     VisionConstants.xyStdDevCoefficient * Math.pow(avgDistance, 2) / tagPoses.size();
    double thetaStdDev =
     VisionConstants.thetaStdDevCoefficient
        * Math.pow(avgDistance, 2)  / tagPoses.size();
  }

   public void odometryWithVision() {
    if (lltv) {
      boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(
          tagLimelightName, m_drive.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              tagLimelightName);
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

    m_field.getObject(tagLimelightName).setPose(m_drive.getPose());
  }
} */
