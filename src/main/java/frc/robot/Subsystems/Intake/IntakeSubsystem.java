package frc.robot.Subsystems.Intake;

import static frc.robot.Constants.Shooter.LOWER_SHOOTER_IDLE_RPM;
import static frc.robot.Constants.Shooter.LOWER_SHOOTER_SPEAKER_RPM;
import static frc.robot.Constants.Shooter.UPPER_SHOOTER_AMP_RPM;
import static frc.robot.Constants.Shooter.UPPER_SHOOTER_IDLE_RPM;
import static frc.robot.Constants.Shooter.UPPER_SHOOTER_SPEAKER_RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LoggedTunableNumber;
import frc.robot.Subsystems.Shooter.ShooterSubsystem.ShooterState;

public class IntakeSubsystem extends SubsystemBase {

  /*Io and inputs */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /*Variables */

  private IntakeStates intakeState = IntakeStates.IDLE;

  private boolean requestIntake = false;

  private String currentIntakeState = "IDLE";

  private double desiredIntakeVel = 0.0;  

      LoggedTunableNumber ampUpperSpeed = new LoggedTunableNumber("Intake/IntakeVel", desiredIntakeVel);


  public enum IntakeStates{
    IDLE,  
    INTAKING
   }

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("Intake", inputs);

    SmartDashboard.putString("Intake State", currentIntakeState);

    if (intakeState == IntakeStates.IDLE){
      io.setVoltage(desiredIntakeVel);
    }
    if (requestIntake){
            intakeState = IntakeStates.INTAKING;
      currentIntakeState = "INTAKING";
    }

    else if (intakeState == IntakeStates.INTAKING){
      io.setVoltage(desiredIntakeVel);

      if(!requestIntake){
        intakeState = IntakeStates.IDLE;
        currentIntakeState = "IDLE";
      }
    }
  }


   public void requestIdle(){
    desiredIntakeVel = 0;
    unsetAllRequests();
  }

  public void requestIntake(double vel){
    desiredIntakeVel = vel;
    unsetAllRequests();
    requestIntake = true;
  }

  public void unsetAllRequests(){
   requestIntake = false;
   intakeState = IntakeStates.IDLE;
  }

}
