package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  public void setIntake(double vel){
    io.setVoltage(vel);
  }

  public void stopIntake(){
    io.setVoltage(0);
  }

  public boolean getSensor(){
    return inputs.sensor;
  }

}
