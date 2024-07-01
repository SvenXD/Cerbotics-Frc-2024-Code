package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.Subsystems.Swerve.GyroIO;
import frc.robot.Subsystems.Swerve.GyroIOInputsAutoLogged;

public class AprilTagLocalizer extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final SwerveDrivePoseEstimator m_poseEstimator;

  public AprilTagLocalizer(GyroIO gyroIO) {
    this.gyroIO = gyroIO;

        m_poseEstimator = new SwerveDrivePoseEstimator(
          Drive.kSwerveKinematics,
         gyroInputs.heading,
          m_drive.getModulePositions(),
          new Pose2d(0,0 ,
          m_drive.getRotation2d()));
  }

  @Override
  public void periodic() {

  }
}
