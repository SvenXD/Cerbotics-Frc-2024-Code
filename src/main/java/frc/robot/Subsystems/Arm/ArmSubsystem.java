package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LoggedTunableNumber;

public class ArmSubsystem extends SubsystemBase {

    /*Io and inputs */
  private final ArmIO io;
  private final ArmIoInputsAutoLogged inputs = new ArmIoInputsAutoLogged();

  /*Variables */
  private double desirableAngle = 0.0;
  private String currentArmState = "IDLE";

  private boolean m_enabled = false;


  LoggedTunableNumber desiredArmAngle = new LoggedTunableNumber("Arm/ArmAngle", desirableAngle);


    private SendableChooser<String> armModeChooser = new SendableChooser<>();
    private String currentModeSelection;
    private final String[] modeNames = {"BRAKE", "COAST"};


  public ArmSubsystem(ArmIO io) {
    this.io = io;

    armModeChooser.setDefaultOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Coast Mode", modeNames[1]);

    SmartDashboard.putData("Arm Mode", armModeChooser);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();

    Logger.processInputs("Arm", inputs);
    SmartDashboard.putString("Arm state", currentArmState);
    SmartDashboard.putNumber("SetPoint", inputs.setPoint);
    SmartDashboard.putNumber("Current Arm Angle",inputs.currentAngle);

    if (m_enabled) {
    io.putThisInPeriodicBecauseOtherwiseItWontWorkAndItsReallyImportant();
    }

  if(DriverStation.isDisabled()){
        currentModeSelection = armModeChooser.getSelected();
        switch (currentModeSelection) {
          case "BRAKE":
            armSetBrake();
          break;

          case "COAST":
            armSetCoast();
          break;
        }
      } else {
        armSetBrake();
      }
    
  }

  public void armSetCoast(){
   io.setCoastMode();
  }

  public void armSetBrake(){
    io.setBrakeMode();
  }

    public Command goToPosition(double position){
    Command ejecutable = Commands.runOnce(
                () -> {
                io.positionFunction(position);  
                enable();        
                },
                this);
    return ejecutable;
  }

  public void enable() {
    m_enabled = true;
  }

      
}
