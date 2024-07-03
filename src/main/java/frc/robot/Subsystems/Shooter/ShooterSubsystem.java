package frc.robot.Subsystems.Shooter; 

import static frc.robot.Constants.Shooter.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LoggedTunableNumber;

public class ShooterSubsystem extends SubsystemBase {

  /*Io and inputs */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    LoggedTunableNumber ampUpperSpeed = new LoggedTunableNumber("UpAmp/ShotVelocity", UPPER_SHOOTER_AMP_RPM);
    LoggedTunableNumber ampLowerSpeed = new LoggedTunableNumber("LowAmp/ShotVelocity", LOWER_SHOOTER_AMP_RPM);


  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("Shooter", inputs);
  }


  public void velocity(double up, double down){
    io.setVelocity(up, down);
  }

  public void stopMotors(){
    io.stop();
  }


}
