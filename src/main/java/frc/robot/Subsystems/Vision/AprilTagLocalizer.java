package frc.robot.Subsystems.Vision;

import frc.Util.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;


public class AprilTagLocalizer extends SubsystemBase {

    private final CommandSwerveDrivetrain m_drive;
  public AprilTagLocalizer(CommandSwerveDrivetrain m_drive) {

    this.m_drive = m_drive;
          
  }

  @Override
  public void periodic() {

  }
  
}