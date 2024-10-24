package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Gripper subsystem hardware interface. */
public interface ArmIO {

  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmIoInputs {

    public double pivotVel = 0.0;
    public double pivotVoltage = 0.0;

    public double currentAngle = 0.0;
    public double setPoint = 0.0;
    public double error = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIoInputs inputs) {}

  /** This keeps the feedfoward needed for the arm to move */
  public default void setDesiredAngle(Rotation2d angle) {}

  /** Enable brake mode on the intake. */
  public default void setBrakeMode(boolean enable) {}

  /** Enable coast mode on the intake. */
  public default void setCoastMode() {}

  /** Run motors at volts */
  public default void setOpenLoop(double v) {}

  /** Stops the motors */
  public default void superSimPeriodic() {}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}
