package frc.robot.Subsystems.IntakexAbajo;

import static frc.robot.Constants.IntakexAb.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.IntakexAbajo.*;

public class IntakexAbIOTalon implements IntakeAbIO {

  private final TalonFX intakeAbMotor = new TalonFX(INTAKE_AB_ID, "rio");

  private TalonFXConfigurator intakeAbConfigurator;

  private CurrentLimitsConfigs intakeAbCurrentLimitsConfigs;
  private MotorOutputConfigs intakeAbOutputConfigs;
  private Slot0Configs intakeAbOuSlot0Configs;

  private final VelocityVoltage intakeAbVelocity = new VelocityVoltage(0);

  private double desiredIntakeAb = 0;

  private StatusSignal<Double> velocityIntakeAb;
  private StatusSignal<Double> positionIntakeAb;
  private StatusSignal<Double> statorIntakeAb;
  private StatusSignal<Double> supplyIntakeAb;

  public IntakexAbIOTalon() {
    intakeAbConfigurator = intakeAbMotor.getConfigurator();

    intakeAbCurrentLimitsConfigs = new CurrentLimitsConfigs();
    intakeAbCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    intakeAbCurrentLimitsConfigs.StatorCurrentLimit = 60;

    intakeAbOutputConfigs = new MotorOutputConfigs();
    intakeAbOutputConfigs.Inverted = INTAKE_AB_INVERSION;
    intakeAbOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    intakeAbConfigurator.apply(intakeAbCurrentLimitsConfigs);
    intakeAbConfigurator.apply(intakeAbOutputConfigs);
    intakeAbConfigurator.apply(intakeAbOuSlot0Configs);

    velocityIntakeAb = intakeAbMotor.getVelocity();
    positionIntakeAb = intakeAbMotor.getPosition();
    statorIntakeAb = intakeAbMotor.getStatorCurrent();
    statorIntakeAb = intakeAbMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, velocityIntakeAb, positionIntakeAb, statorIntakeAb, supplyIntakeAb);
  }

  @Override
  public void updateInputs(IntakeAbIOInputs inputs) {

    BaseStatusSignal.refreshAll(velocityIntakeAb, positionIntakeAb, statorIntakeAb, supplyIntakeAb);
    /* Upper inputs */
    inputs.positionRad = Units.rotationsToRadians(positionIntakeAb.getValue());
    inputs.velocityRadPerSec = velocityIntakeAb.getValueAsDouble() * 60;
    inputs.appliedVolts = intakeAbMotor.getMotorVoltage().getValue();
    inputs.tempCelcius = intakeAbMotor.getDeviceTemp().getValue();
  }
  
  @Override 
  public void stop(){
    intakeAbMotor.stopMotor();
  }

  @Override
  public void setVelocity(double velocityIntakeAb){
    intakeAbVelocity.Velocity = velocityIntakeAb / 60;
    intakeAbMotor.setControl(intakeAbVelocity);
  }

  @Override
  public void setVoltage(double volts){
    intakeAbMotor.setControl(new VoltageOut(volts).withEnableFOC(true));
  }
}
