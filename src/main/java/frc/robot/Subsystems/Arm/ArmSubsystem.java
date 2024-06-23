package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LoggedTunableNumber;

public class ArmSubsystem extends SubsystemBase {

    /*Io and inputs */
  private final ArmIO io;
  private final ArmIoInputsAutoLogged inputs = new ArmIoInputsAutoLogged();

  /*Variables */
  private double desirableAngle = 0.0;
  LoggedTunableNumber desiredArmAngle = new LoggedTunableNumber("Arm/ArmAngle", desirableAngle);


  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("Arm", inputs);
  }


}
