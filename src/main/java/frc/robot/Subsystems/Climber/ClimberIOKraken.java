package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOKraken implements ClimberIO {

  private final TalonFX climberMotor = new TalonFX(51, "rio");
  private final TalonFXConfiguration climberConfig = new TalonFXConfiguration();

  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  public ClimberIOKraken() {
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberConfig.CurrentLimits.StatorCurrentLimit = 60;
    climberConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    climberConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    climberMotor.setPosition(0);

    climberMotor.getConfigurator().apply(climberConfig);
  }

  @Override
  public void setVoltage(double voltage) {
    climberMotor.set(voltage);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = climberMotor.getMotorVoltage().getValue();
    inputs.tempCelcius = climberMotor.getDeviceTemp().getValueAsDouble();
    inputs.position = climberMotor.getPosition().getValueAsDouble();
  }
}
