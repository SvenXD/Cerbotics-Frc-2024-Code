package frc.robot.Subsystems.Pivot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double pivotVel = 0.0;
    public double pivotSetpoint = 0.0;
    public double pivotPositionDeg = 0.0;
    public double pivotVoltage = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Run the shooter closed loop at the specified velocity. */
  public default void setOpenLoop(double v) {}

  public default void setDesiredAngle(Measure<Angle> angle) {}

  /** Stops the shooter. */
  public default void stop() {}

  public default void superSimPeriodic() {}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}
