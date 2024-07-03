
package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;


public interface AprilTagIO {
    @AutoLog
    public static class AprilTagIOInputs{
        public double distanceFromTarget = 0.0;
        public double numOfTagsDetected = 0.0;
        public double tX = 0.0;
        public double tY = 0.0;
        public boolean tV = false;

        public int currentPipeline = 0;
        public Pose3d test;
    }

   /** Updates the set of loggable inputs. */
  public default void updateInputs(AprilTagIOInputs inputs) {}

  public default void changeSmartPipeline(){}

}
