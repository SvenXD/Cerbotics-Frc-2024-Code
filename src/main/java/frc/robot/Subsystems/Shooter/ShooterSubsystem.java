package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LoggedTunableNumber;

public class ShooterSubsystem extends SubsystemBase {

  /*Io and inputs */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /* Variables */

  private ShooterState systemState = ShooterState.IDLE;

  private boolean requestSpeaker = false;
  private boolean requestAMP = false;
  private boolean requestLow_pass = false;
  private boolean requestHigh_Pass = false;

  /* Setpoints*/

  private double desiredUpperRPM = 1000.0;
  private double desiredLowerRPM = 1000.0;

    LoggedTunableNumber ampSpeed = new LoggedTunableNumber("Amp/ShotVelocity", desiredLowerRPM);



     /* Shooter states */
   public enum ShooterState{
    IDLE,  
    SPEAKER,
    AMP,
    LOW_PASS,
    HIGH_PASS
   }

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("Shooter", inputs);

   // Logger.recordOutput("Shooter State", null);
  }

  public void requestIdle(){
  
  }

  public void requestSpeaker(){

  }

  public void requestAMP(){

  }

  public void requestLow_pass(){

  }

  public void requestHigh_Pass(){

  }
}
