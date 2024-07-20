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
    public double setPoint = 0.0;
    public double error = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIoInputs inputs) {}

   /** This keeps the feedfoward needed for the arm to move */
   public default void setVoltage(double output, double feedfoward){}

   /** Enable brake mode on the intake. */
   public default void setBrakeMode() {}

   /** Enable coast mode on the intake. */
   public default void setCoastMode() {}

   /** Run motors at volts */
   public default void runVolts(double volts) {}

   /**Stops the motors */
   public default void stop(){}

  /* Updates the tunable numbers. */
  public default void updateTunableNumbers() {}
}