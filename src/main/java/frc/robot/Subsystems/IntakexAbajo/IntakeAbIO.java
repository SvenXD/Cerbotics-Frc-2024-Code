package frc.robot.Subsystems.IntakexAbajo;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeAbIO {
  @AutoLog
  public static class IntakeAbIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double tempCelcius = 0.0;

    public boolean sensor = false;
  }

 /** Updates the set of loggable inputs. */
 public default void updateInputs(IntakeAbIOInputs inputs) {}

 public default void updateTunableNumbers() {}

 /** Run the intake open loop at the specified voltage. */
 public default void setVoltage(double volts) {}

 public default void stop() {}

 public default void setVelocity(double velocityIntakeAb) {}

}
