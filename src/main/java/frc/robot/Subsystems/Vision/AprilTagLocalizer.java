package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;
import static frc.robot.Constants.VisionConstants.*;

import org.littletonrobotics.junction.Logger;


public class AprilTagLocalizer extends SubsystemBase {

    /*Io and inputs */
  private final AprilTagIO io;
  private final AprilTagIOInputsAutoLogged inputs = new AprilTagIOInputsAutoLogged();

  Field2d m_field = new Field2d();

  private final CommandSwerveDrivetrain m_drive;
  public AprilTagLocalizer(CommandSwerveDrivetrain m_drive,AprilTagIO io) {
    this.io = io;
    this.m_drive = m_drive;
          
  }

  @Override
  public void periodic() {
    odometryWithVision();
    io.changeSmartPipeline();
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

  }

  public void odometryWithVision(){
    if(inputs.tV){
      boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(tagLimelightName  ,m_drive.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tagLimelightName);
      if(Math.abs(m_drive.getPigeon2().getAngle()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_drive.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));
        m_drive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
    else{
      m_field.getObject(tagLimelightName).setPose(m_drive.getState().Pose);
    }
  }

  
  
}