package frc.robot.Subsystems.Shooter;

import static frc.robot.Constants.Shooter.*;

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

  private double desiredUpperRPM = 0.0;
  private double desiredLowerRPM = 0.0;

    LoggedTunableNumber ampUpperSpeed = new LoggedTunableNumber("Amp/ShotVelocity", UPPER_SHOOTER_AMP_RPM);
        LoggedTunableNumber ampLowerSpeed = new LoggedTunableNumber("Amp/ShotVelocity", LOWER_SHOOTER_AMP_RPM);


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

   ShooterState nextSystemState = systemState;

    if (systemState == ShooterState.IDLE){
      io.setVelocity(desiredUpperRPM, desiredLowerRPM);
    }
    if(requestSpeaker){
      nextSystemState = ShooterState.SPEAKER;
    }
    else if(requestAMP){
      nextSystemState = ShooterState.AMP;
    }
    else if(requestHigh_Pass){
      nextSystemState = ShooterState.HIGH_PASS;
    }
    else if(requestLow_pass){
      nextSystemState = ShooterState.LOW_PASS;
    }

    else if (systemState == ShooterState.SPEAKER){
      io.setVelocity(desiredUpperRPM, desiredLowerRPM);

      if(!requestSpeaker){
        nextSystemState = ShooterState.IDLE;
      }
    }
    else if (systemState == ShooterState.AMP){
      io.setVelocity(desiredUpperRPM, desiredLowerRPM);

      if(!requestAMP){
        nextSystemState = ShooterState.IDLE;
      }
    }
    else if (systemState == ShooterState.HIGH_PASS){
      io.setVelocity(desiredUpperRPM, desiredLowerRPM);

      if(!requestHigh_Pass){
        nextSystemState = ShooterState.IDLE;
      }
    }
    else if (systemState == ShooterState.LOW_PASS){
      io.setVelocity(desiredUpperRPM, desiredLowerRPM);

      if(!requestLow_pass){
        nextSystemState = ShooterState.IDLE;
      }
    }
  }

  public void requestIdle(){
    desiredUpperRPM = UPPER_SHOOTER_IDLE_RPM;
    desiredLowerRPM = LOWER_SHOOTER_IDLE_RPM;
    unsetAllRequests();
  }

  public void requestSpeaker(double upperSpeed, double lowerSpeed){
    desiredUpperRPM = upperSpeed;
    desiredLowerRPM = lowerSpeed;
    unsetAllRequests();
    requestSpeaker = true;
  }

  public void requestAMP(){
    desiredUpperRPM = ampUpperSpeed.get();
    desiredLowerRPM = ampLowerSpeed.get();
    unsetAllRequests();
    requestAMP = true;
  }

  public void requestLow_pass(){
    desiredUpperRPM = UPPER_SHOOTER_FEEDER_UNDER_RPM;
    desiredLowerRPM = UPPER_SHOOTER_FEEDER_UNDER_RPM;
    unsetAllRequests();
    requestLow_pass = true;
  }

  public void requestHigh_Pass(){
    desiredUpperRPM = UPPER_SHOOTER_FEEDER_OVER_RPM;
    desiredLowerRPM = LOWER_SHOOTER_FEEDER_OVER_RPM;
    unsetAllRequests();
    requestHigh_Pass = true;
  }

  public void unsetAllRequests(){

   requestSpeaker = false;
   requestAMP = false;
   requestLow_pass = false;
   requestHigh_Pass = false;
  }
}
