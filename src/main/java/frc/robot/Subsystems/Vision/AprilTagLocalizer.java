package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.robot.Subsystems.Swerve.Drive;

import static frc.robot.Constants.VisionConstants.*;

import org.littletonrobotics.junction.Logger;


public class AprilTagLocalizer extends SubsystemBase {

    /*Io and inputs */
  private final AprilTagIO io;
  private final AprilTagIOInputsAutoLogged inputs = new AprilTagIOInputsAutoLogged();

  Field2d m_field = new Field2d();

  private final Drive drive;

  public AprilTagLocalizer(Drive drive ,AprilTagIO io) {
    this.io = io;
    this.drive = drive;
          
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
      LimelightHelpers.SetRobotOrientation(tagLimelightName  ,drive.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tagLimelightName);
      if(Math.abs(drive.getangle()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        drive.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));
        drive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
    else{
      m_field.getObject(tagLimelightName).setPose(drive.getPose());
    }
  }

  
  
}