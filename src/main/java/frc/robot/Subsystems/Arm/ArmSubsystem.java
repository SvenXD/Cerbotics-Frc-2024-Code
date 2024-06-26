package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.LoggedTunableNumber;
import frc.robot.Subsystems.Intake.IntakeSubsystem.IntakeStates;

public class ArmSubsystem extends SubsystemBase {

    /*Io and inputs */
  private final ArmIO io;
  private final ArmIoInputsAutoLogged inputs = new ArmIoInputsAutoLogged();

  /*Variables */
  private double desirableAngle = 0.0;
  private ArmStates armState = ArmStates.IDLE;
  private String currentArmState = "IDLE";

  LoggedTunableNumber desiredArmAngle = new LoggedTunableNumber("Arm/ArmAngle", desirableAngle);

  public enum ArmStates{
    IDLE,  
    Test
   }

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
    SmartDashboard.putNumber("SetPointa", inputs.setPoint);
    SmartDashboard.putNumber("Current Arm Angle", Units.radiansToDegrees(inputs.currentAngle));


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

  public void test(){
    io.goToPosition(92);
  }

  public void armSetBrake(){
    io.setBrakeMode();
  }

}
