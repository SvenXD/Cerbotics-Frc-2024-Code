package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double upperShooterPosRad = 0.0;
    public double upperShooterVelocityRpm = 0.0;
    public double upperShooterAppliedVolts = 0.0;
    public double upperShooterTempCelcius = 0.0;
    public double upperShooterSetPointRpm = 0.0;

    public double lowerShooterPosRad = 0.0;
    public double lowerShooterVelocityRpm = 0.0;
    public double lowerShooterAppliedVolts = 0.0;
    public double lowerShooterTempCelcius = 0.0;
    public double lowerShooterSetPointRpm = 0.0;

    public double[] shooterCurrentAmps = new double[] {}; // {upper, lower}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the shooter closed loop at the specified velocity. */
  public default void setVelocity(double velocityRpmUpper, double velocityRpmLower) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stops the shooter. */
  public default void stop() {}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}
