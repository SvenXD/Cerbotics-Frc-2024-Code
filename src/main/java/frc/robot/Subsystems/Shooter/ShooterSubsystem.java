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

  private ShooterState systemState = ShooterState.NOTINNITIALICED;

  private boolean requestSpeaker = false;
  private boolean requestAMP = false;
  private boolean requestLow_pass = false;
  private boolean requestHigh_Pass = false;
  private boolean requestCustom = false;

  private boolean test = false;

  private String shoterState = "IDLE";

  /* Setpoints*/

  private double desiredUpperRPM = 0.0;
  private double desiredLowerRPM = 0.0;

    LoggedTunableNumber ampUpperSpeed = new LoggedTunableNumber("UpAmp/ShotVelocity", UPPER_SHOOTER_AMP_RPM);
        LoggedTunableNumber ampLowerSpeed = new LoggedTunableNumber("LowAmp/ShotVelocity", LOWER_SHOOTER_AMP_RPM);


     /* Shooter states */
   public enum ShooterState{
    NOTINNITIALICED,
    IDLE,  
    SPEAKER,
    AMP,
    LOW_PASS,
    HIGH_PASS,
    CUSTOM
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
    SmartDashboard.putBoolean("test", test);
  }

  public void requestIdle(){
    desiredUpperRPM = UPPER_SHOOTER_IDLE_RPM;
    desiredLowerRPM = LOWER_SHOOTER_IDLE_RPM;
    unsetAllRequests();
  }

  public void test(double up, double down){
    io.setVelocity(up, down);
    test = true;
  }

  public void test2(){
    io.stop();
    test = false;
  }

  public void requestSpeaker(){
    desiredUpperRPM = UPPER_SHOOTER_SPEAKER_RPM;
    desiredLowerRPM = LOWER_SHOOTER_SPEAKER_RPM;
    requestSpeaker = true;
  }

  public void requestAMP(){
    desiredUpperRPM = UPPER_SHOOTER_AMP_RPM;
    desiredLowerRPM = LOWER_SHOOTER_AMP_RPM;
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
