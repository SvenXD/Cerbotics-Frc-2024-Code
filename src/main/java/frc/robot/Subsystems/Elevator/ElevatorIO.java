package frc.robot.Subsystems.Elevator;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {

    public double simPivotPositionDeg = 0.0;
    public double simPivotVoltage = 0.0;

    public double pivotVel = 0.0;
    public double pivotSetpoint = 0.0;
    public double pivotPositionDeg = 0.0;
    public double pivotVoltage = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setOpenLoop(double v) {}

  public default void setDesiredAngle(Measure<Angle> angle) {}

  public default void superSimPeriodic() {}

  public default void setBrake(boolean brake) {}

  public default void superPeriodic(){}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}
