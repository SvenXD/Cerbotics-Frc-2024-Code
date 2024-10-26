package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmInputs {
    public double angleDegrees = 0.0;
    public double closedLoopError = 0.0;

    public double leftVelocityDps = 0.0;
    public double leftAccelDpsSq = 0.0;

    public double rightVelocityDps = 0.0;
    public double rightAccelDpsSq = 0.0;

    public double leftMotorTemp = 0.0;
    public double leftMotorSupplyCurrent = 0.0;
    public double leftMotorStatorCurrent = 0.0;
    public double leftMotorVoltage = 0.0;
    public double leftMotorSupplyVoltage = 0.0;

    public double rightMotorTemp = 0.0;
    public double rightMotorSupplyCurrent = 0.0;
    public double rightMotorStatorCurrent = 0.0;
    public double rightMotorVoltage = 0.0;
    public double rightMotorSupplyVoltage = 0.0;
  }

  /**
   * Update the inputs for the arm joint
   *
   * @param inputs The inputs to update
   */
  void updateInputs(ArmInputs inputs);

  /**
   * Set the target angle for the arm joint
   *
   * @param angle Target arm angle
   */
  void setTargetAngle(Rotation2d angle);

  void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel);

  /**
   * Set the voltage output to the arm joint motors
   *
   * @param volts Voltage to output
   */
  void setVoltage(double volts);

  /** Optimize status signals for running sysID */
  default void optimizeForSysID() {}

  /**
   * Get a list of all devices to be used for orchestra commands
   *
   * @return Orchestra compatible CTRE devices
   */
  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }
}
