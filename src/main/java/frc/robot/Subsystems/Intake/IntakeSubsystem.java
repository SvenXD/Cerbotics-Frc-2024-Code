package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  /*Io and inputs */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setUpperIntakeVoltageVoid(double voltage) {
    io.setVoltage(voltage);
  }

  public void setLowerIntakeVoltageVoid(double voltage) {
    io.lowerIntakeSet(voltage);
  }

  public void setallVoid(double voltage, double lup) {
    io.setAll(voltage, lup);
  }

  public Command setlowerIntakeVoltage(double voltage) {
    return run(() -> io.lowerIntakeSet(voltage));
  }

  public Command setUpperVoltage(double voltage) {
    return run(() -> io.setVoltage(voltage));
  }

  public Command setall(double voltage, double lup) {
    return run(() -> io.setAll(voltage, lup));
  }

  public boolean isNoteInside() {
    return inputs.sensor;
  }
}
