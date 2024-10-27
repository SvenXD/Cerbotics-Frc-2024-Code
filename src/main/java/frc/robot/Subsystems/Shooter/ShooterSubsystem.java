package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  /*Io and inputs */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("Shooter", inputs);
  }

  public Command setRpms(double upperRpm, double downRpm) {
    return run(() -> io.setVelocity(upperRpm, downRpm));
  }

  public Command stop() {
    return run(() -> io.stop());
  }

  public Command setPower() {
    return run(() -> io.setVoltage(0.3));
  }
}
