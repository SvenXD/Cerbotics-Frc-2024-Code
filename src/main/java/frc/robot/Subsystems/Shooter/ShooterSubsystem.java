package frc.robot.Subsystems.Shooter;

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
    Logger.recordOutput("Shooter/RPMS", inputs.lowerShooterAppliedVolts);
  }

  public void velocity(double up, double down) {
    io.setVelocity(up, down);
  }

  public void stopMotors() {
    io.stop();
  }
}
