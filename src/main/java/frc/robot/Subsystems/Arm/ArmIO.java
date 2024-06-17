package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

/** Gripper subsystem hardware interface. */
public interface ArmIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double tempCelcius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake open loop at the specified voltage. */
  public default void setPosition(int angle) {}

  /** Enable or disable brake mode on the intake. */
  public default void setBrakeMode(boolean enable) {}

  /** Set the limit of current to the moro */
  public default void setCurrentLimit(int currentLimit) {}

    /* Updates the tunable numbers. */
    public default void updateTunableNumbers() {}
}