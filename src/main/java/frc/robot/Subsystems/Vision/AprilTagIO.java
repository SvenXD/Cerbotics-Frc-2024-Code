
package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.Swerve.Drive;


public interface AprilTagIO {
    @AutoLog
    public static class AprilTagIOInputs{
        public double distanceFromTarget = 0.0;
        public double numOfTagsDetected = 0.0;
        public double tX = 0.0;
        public double tY = 0.0;
        public boolean tV = false;

        public int currentPipeline = 0;
    }

   /** Updates the set of loggable inputs. */
  public default void updateInputs(AprilTagIOInputs inputs) {}

  public default void changeSmartPipeline(){}

  public default void ActivateSimParameters(Pose2d robotPoseMeters){}

  public default void measurements(Drive m_drive){}

}
