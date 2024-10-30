package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeIOKraken implements IntakeIO {

  /* Hardware */
  private final TalonFX intakeMotor = new TalonFX(52, "rio");
  private final TalonFX lUpMotor = new TalonFX(17, "rio");
  private final TalonFX lDownMotor = new TalonFX(18, "rio");
  private final DigitalInput intakeSensor = new DigitalInput(Constants.Intake.INTAKE_SENSOR_ID);

  /* Configurators */
  private TalonFXConfiguration intakeConfig;

  /* Status signals */
  private StatusSignal<Double> temperature;
  private StatusSignal<Double> supply;

  public IntakeIOKraken() {

    lUpMotor.setInverted(true);
    lDownMotor.setInverted(true);

    intakeConfig = new TalonFXConfiguration();

    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
    intakeConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    intakeConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    /* Apply Configurations*/
    intakeMotor.getConfigurator().apply(intakeConfig);

    /* Status Signals */

    temperature = intakeMotor.getDeviceTemp();
    supply = intakeMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50, temperature, supply);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(temperature, supply);

    inputs.appliedVolts = supply.getValueAsDouble();
    inputs.tempCelcius = temperature.getValueAsDouble();
    inputs.sensor = !intakeSensor.get();
  }

  @Override
  public void setVoltage(double armVolt) {
    intakeMotor.set(-armVolt);
  }

  @Override
  public void lowerIntakeSet(double voltage) {
    lDownMotor.set(voltage);
    lUpMotor.set(voltage);
  }

  @Override
  public void setAll(double voltage, double lup) {
    lDownMotor.set(voltage);
    lUpMotor.set(lup);
    intakeMotor.set(-voltage);
  }
}
