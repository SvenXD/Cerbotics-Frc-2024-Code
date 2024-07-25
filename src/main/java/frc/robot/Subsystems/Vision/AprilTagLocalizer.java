package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
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
          
    SmartDashboard.putData("Field", m_field);

  }

  @Override
  public void periodic() {
        //updateOdometry();
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    io.ActivateSimParameters(drive.getPose());
    io.measurements(drive);
  }

  public void updateSimOdometry(){

  }
/** Updates the field relative position of the robot. */
  /*public void updateOdometry() {

    drive.updatePoseEstimator();
    boolean doRejectUpdate = false;

    if(inputs.tV){
      LimelightHelpers.SetRobotOrientation(tagLimelightName, drive.getSwerveDrivePoseEstimator().getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tagLimelightName);
      if(Math.abs(drive.getangle()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        drive.getSwerveDrivePoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        drive.getSwerveDrivePoseEstimator().addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
      }
      else{
      m_field.getObject(tagLimelightName).setPose(drive.getPose());
      }
    }

  */
  
}