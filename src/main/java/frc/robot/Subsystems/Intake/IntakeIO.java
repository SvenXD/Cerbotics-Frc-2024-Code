package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ProfiledPIDController;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs{
        public double positionDegrees = 0.0;
        public double intakeCurrentAmps = 0.0;
        }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}
  
  public default void setIntakePosition(double position, double feedfoward){}
    
}
