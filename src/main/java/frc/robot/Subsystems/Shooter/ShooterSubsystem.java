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

  /* Variables */

  private ShooterState systemState = ShooterState.SPEAKER;

  private boolean requestSpeaker = false;
  private boolean requestAMP = false;
  private boolean requestLow_pass = false;
  private boolean requestHigh_Pass = false;
  private boolean requestCustom = false;

  private boolean requested = false;

  private String shoterState = "IDLE";

  /* Setpoints*/

  private double desiredUpperRPM = 0;
  private double desiredLowerRPM = 0;

    LoggedTunableNumber ampUpperSpeed = new LoggedTunableNumber("ShotUp/ShotVelocity", desiredUpperRPM);
        LoggedTunableNumber ampLowerSpeed = new LoggedTunableNumber("ShotLow/ShotVelocity", desiredLowerRPM);


     /* Shooter states */
   public enum ShooterState{
    IDLE,  
    SPEAKER,
    AMP,
    LOW_PASS,
    HIGH_PASS,
    CUSTOM,
    test
   }

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("Shooter", inputs);

    SmartDashboard.putString("Shooter State", shoterState);
    SmartDashboard.putBoolean("oewn", requested);




    if(requestSpeaker){
           systemState = ShooterState.SPEAKER;
      shoterState = "SPEAKER";
    }
    else if(requestAMP){
            systemState = ShooterState.AMP;
      shoterState = "AMP";
    }
    else if(requestHigh_Pass){
           systemState = ShooterState.HIGH_PASS;
      shoterState = "HIGH PASS";

    }
    else if(requestLow_pass){
            systemState = ShooterState.LOW_PASS;
      shoterState = "LOW PASS"; 

    }
        else if(requestCustom){
            systemState = ShooterState.CUSTOM;
      shoterState = "CUSTOM"; 

    }

    else if (systemState == ShooterState.SPEAKER){
      io.setVelocity(desiredUpperRPM, desiredLowerRPM);

      if(!requestSpeaker){
        systemState = ShooterState.test;
        shoterState = "plea34mise work";
      }
    }
    else if (systemState == ShooterState.AMP){
      io.setVelocity(desiredUpperRPM, desiredLowerRPM);

      if(!requestAMP){
        systemState = ShooterState.test;
        shoterState = "please work";
      }
    }
    else if (systemState == ShooterState.HIGH_PASS){
      io.setVelocity(desiredUpperRPM, desiredLowerRPM);

      if(!requestHigh_Pass){
        systemState = ShooterState.test;
        shoterState = "please work";
      }
    }
    else if (systemState == ShooterState.LOW_PASS){
      io.setVelocity(desiredUpperRPM, desiredLowerRPM);

      if(!requestLow_pass){
        systemState = ShooterState.test;
        shoterState = "please work";
      }
    }
    else if (systemState == ShooterState.CUSTOM){
      io.setVelocity(desiredUpperRPM, desiredLowerRPM);

      if(!requestCustom){
        systemState = ShooterState.test;
        shoterState = "please work";
      }
    }
          

  }

  public void requestIdle(){
    desiredUpperRPM = 0;
    desiredLowerRPM = 0;
    unsetAllRequests();
  }

  public void requestSpeaker(){
    desiredUpperRPM = UPPER_SHOOTER_SPEAKER_RPM;
    desiredLowerRPM = LOWER_SHOOTER_SPEAKER_RPM;
    unsetAllRequests();
    requestSpeaker = true;
        requested = true;

  }

  public void requestAMP(){
    desiredUpperRPM = UPPER_SHOOTER_AMP_RPM;
    desiredLowerRPM = LOWER_SHOOTER_AMP_RPM;
    unsetAllRequests();
    requestAMP = true;
    requested = false;
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

  public void requestCustom(double upper, double lower){
    desiredUpperRPM = upper;
    desiredLowerRPM = lower;
    unsetAllRequests();
    requestCustom = true;

  }

    public void unsetAllRequests(){
    requestSpeaker = false;
    requestAMP = false;
    requestLow_pass = false;
    requestHigh_Pass = false;
    requestCustom = false;
    }
}
