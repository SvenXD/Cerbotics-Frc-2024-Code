package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    /*Io and inputs */
  private final ArmIO io;
  private final ArmIoInputsAutoLogged inputs = new ArmIoInputsAutoLogged();

  /*Variables */

  private boolean m_enabled = false;

    private SendableChooser<String> armModeChooser = new SendableChooser<>();
    private String currentModeSelection;
    private final String[] modeNames = {"BRAKE", "COAST"};


  public ArmSubsystem(ArmIO io) {
    this.io = io;
    io.updateTunableNumbers();
    Logger.processInputs("Arm", inputs);

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
 

    if (m_enabled) {
    io.putThisInPeriodicBecauseOtherwiseItWontWorkAndItsReallyImportant();
    }

  if(DriverStation.isDisabled()){
        currentModeSelection = armModeChooser.getSelected();
        switch (currentModeSelection) {
          case "BRAKE":
            io.setBrakeMode();
          break;

          case "COAST":
            io.setCoastMode();
          break;
        }
      } else {
       io.setBrakeMode();
      }
    
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
