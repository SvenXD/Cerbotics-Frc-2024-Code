package frc.robot.Subsystems.ArmExtention;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

/** Gripper subsystem hardware interface. */
public interface ArmExtentionIO {

  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmExtentionIOInputs {
    public double appliedVolts = 0.0;
    public double tempCelcius = 0.0;

    public double currentPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmExtentionIOInputs inputs) {}

   /** This keeps the feedfoward needed for the arm to move */
   public default void setVoltage(ProfiledPIDController m_controller, ElevatorFeedforward m_feedforward){}

   /** Run motors at volts */
   public default void runVolts(double volts) {}

   public default void update(double angleRads){}

}