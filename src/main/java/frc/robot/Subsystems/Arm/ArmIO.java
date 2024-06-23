package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

/** Gripper subsystem hardware interface. */
public interface ArmIO {

  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmIoInputs {
    public double leftAppliedVolts = 0.0;
    public double leftTempCelcius = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightTempCelcius = 0.0;
    public double currentAngle = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIoInputs inputs) {}

    /* Updates the tunable numbers. */
    public default void updateTunableNumbers() {}
}