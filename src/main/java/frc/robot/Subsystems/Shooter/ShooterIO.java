package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs{
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
  public default void setVelocity(double velocityRpmUpper, double velocityRpmLower  ) {}

  /** Stops the shooter.  i mean what else do you expect */
  public default void stop(){}

  /* Sets current limit for the flywheel motors. */
  public default void setCurrentLimit(
      double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold) {}

  /* Enables or disables flywheel brake mode. */
  public default void enableBrakeMode(boolean enable) {}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}