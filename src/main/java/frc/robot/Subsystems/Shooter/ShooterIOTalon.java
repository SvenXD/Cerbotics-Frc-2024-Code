package frc.robot.Subsystems.Shooter;

import static frc.robot.Constants.Shooter.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.Util.LoggedTunableNumber;

public class ShooterIOTalon implements ShooterIO {

  /* Hardware */
  private final TalonFX upperMotor = new TalonFX(UPPER_SHOOTER_ID,"rio");
  private final TalonFX lowerMotor = new TalonFX(LOWER_SHOOTER_ID,"rio");

  /* Configurators */
  private TalonFXConfigurator upperConfigurator;
  private TalonFXConfigurator lowerConfigurator;

  private CurrentLimitsConfigs shooterCurrentLimitsConfigs;
  private MotorOutputConfigs upperMotorOutputConfigs;
  private MotorOutputConfigs lowerMotorOutputConfigs;
  private Slot0Configs upperMotorSlot0Configs;
  private Slot0Configs lowerMotorSlot0Configs;

  private final VelocityVoltage upperVelocity = new VelocityVoltage(0);
  private final VelocityVoltage lowerVelocity = new VelocityVoltage(0);

  /* SetPoints */
  private double desiredUp = 0;
  private double desiredDown = 0;

  /* Status signals */
  private StatusSignal<Double> velocityUp;
  private StatusSignal<Double> velocityDown;
  private StatusSignal<Double> positionUp;
  private StatusSignal<Double> positionDown;
  private StatusSignal<Double> statorUp;
  private StatusSignal<Double> statorDown;
  private StatusSignal<Double> supplyUp;
  private StatusSignal<Double> supplyDown;

  LoggedTunableNumber upperShooterKs = new LoggedTunableNumber("UpperShooter/ukS", 0.0);
  LoggedTunableNumber upperShooterKa = new LoggedTunableNumber("UpperShooter/ukA", 0.0);
  LoggedTunableNumber upperShooterKv = new LoggedTunableNumber("UpperShooter/ukV", 0.0);
  LoggedTunableNumber upperShooterKp = new LoggedTunableNumber("UpperShooter/ukP", 0.2);
  LoggedTunableNumber upperShooterKi = new LoggedTunableNumber("UpperShooter/ukI", 0.0);
  LoggedTunableNumber upperShooterKd = new LoggedTunableNumber("UpperShooter/ukD", 0.0);

  LoggedTunableNumber lowerShooterKs = new LoggedTunableNumber("LowerShooter/lkS", 0.0);
  LoggedTunableNumber lowerShooterKa = new LoggedTunableNumber("LowerShooter/lkA", 0.0);
  LoggedTunableNumber lowerShooterKv = new LoggedTunableNumber("LowerShooter/lkV", 0.0);
  LoggedTunableNumber lowerShooterKp = new LoggedTunableNumber("LowerShooter/lkP", 0.2);
  LoggedTunableNumber lowerShooterKi = new LoggedTunableNumber("LowerShooter/lkI", 0.0);
  LoggedTunableNumber lowerShooterKd = new LoggedTunableNumber("LowerShooter/lkD", 0.0);

  public ShooterIOTalon() {

    upperConfigurator = upperMotor.getConfigurator();
    lowerConfigurator = lowerMotor.getConfigurator();


    /* Configurations*/

    /* Current Limit Configuration */
    shooterCurrentLimitsConfigs = new CurrentLimitsConfigs();
    shooterCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    shooterCurrentLimitsConfigs.StatorCurrentLimit = 60;
    shooterCurrentLimitsConfigs.SupplyCurrentLimit = 60;

    /* Motor Output */
    upperMotorOutputConfigs = new MotorOutputConfigs();
    upperMotorOutputConfigs.Inverted = UPPER_SHOOTER_INVERSION;
    upperMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
    upperMotorOutputConfigs.PeakReverseDutyCycle = -1.0;
    upperMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    lowerMotorOutputConfigs = new MotorOutputConfigs();
    lowerMotorOutputConfigs.Inverted = LOWER_SHOOTER_INVERSION;
    lowerMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
    lowerMotorOutputConfigs.PeakReverseDutyCycle = -1.0;
    lowerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    /* Slot 0 configuration */
    upperMotorSlot0Configs = new Slot0Configs();
    upperMotorSlot0Configs.kA = upperShooterKa.get();
    upperMotorSlot0Configs.kS = upperShooterKs.get();
    upperMotorSlot0Configs.kV = upperShooterKv.get();
    upperMotorSlot0Configs.kP = upperShooterKp.get();
    upperMotorSlot0Configs.kI = upperShooterKi.get();
    upperMotorSlot0Configs.kD = upperShooterKd.get();

    lowerMotorSlot0Configs = new Slot0Configs();
    lowerMotorSlot0Configs.kA = lowerShooterKa.get();
    lowerMotorSlot0Configs.kS = lowerShooterKs.get();
    lowerMotorSlot0Configs.kV = lowerShooterKv.get();
    lowerMotorSlot0Configs.kP = lowerShooterKp.get();
    lowerMotorSlot0Configs.kI = lowerShooterKi.get();
    lowerMotorSlot0Configs.kD = lowerShooterKd.get();

    /* Apply Configurations*/
    upperConfigurator.apply(shooterCurrentLimitsConfigs);
    upperConfigurator.apply(upperMotorOutputConfigs);
    upperConfigurator.apply(upperMotorSlot0Configs);

    lowerConfigurator.apply(shooterCurrentLimitsConfigs);
    lowerConfigurator.apply(lowerMotorOutputConfigs);
    lowerConfigurator.apply(lowerMotorSlot0Configs);

    /* Status Signals */

    velocityUp = upperMotor.getVelocity();
    velocityDown = lowerMotor.getVelocity();
    positionUp = upperMotor.getPosition();
    positionDown = lowerMotor.getPosition();
    statorUp = upperMotor.getStatorCurrent();
    statorDown = lowerMotor.getStatorCurrent();
    supplyUp = upperMotor.getSupplyCurrent();
    supplyDown = lowerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
      50, 
      velocityUp,
      velocityDown,
      positionUp,
      positionDown,
      statorUp,
      statorDown,
      supplyUp,
      supplyDown);

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
      BaseStatusSignal.refreshAll(
        velocityUp, 
        velocityDown, 
        positionUp, 
        positionDown, 
        statorUp, 
        statorDown, 
        supplyUp, 
        supplyDown);
      /* Upper inputs */
      inputs.upperShooterPosRad =
            Units.rotationsToRadians(positionUp.getValue());
      inputs.upperShooterVelocityRpm = velocityUp.getValueAsDouble() * 60;
      inputs.upperShooterAppliedVolts = upperMotor.getMotorVoltage().getValue();
      inputs.upperShooterTempCelcius = upperMotor.getDeviceTemp().getValue();
      inputs.upperShooterSetPointRpm = desiredUp;

      /* Lower inputs */
      inputs.lowerShooterPosRad =
            Units.rotationsToRadians(positionDown.getValue());
      inputs.lowerShooterVelocityRpm = velocityDown.getValueAsDouble()*60;
      inputs.lowerShooterAppliedVolts = lowerMotor.getMotorVoltage().getValue();
      inputs.lowerShooterTempCelcius = lowerMotor.getDeviceTemp().getValue();
      inputs.lowerShooterSetPointRpm = desiredDown;

      inputs.shooterCurrentAmps = new double[]{supplyUp.getValue(),supplyDown.getValue()};
  }

  @Override
  public void setPercent(double percentUpper, double percentLower){
    upperMotor.setControl(new DutyCycleOut(percentUpper));
    lowerMotor.setControl(new DutyCycleOut(percentLower));
  }

  @Override
  public void setVelocity(double velocityUp, double velocityDown){
    upperVelocity.Velocity = velocityUp / 60;
    upperMotor.setControl(upperVelocity);

    lowerVelocity.Velocity = velocityDown / 60;
    lowerMotor.setControl(lowerVelocity);
  }

  @Override
  public void setCurrentLimit(  
    double currentLimit, double supplyCurrentThreshold, double supplyTimeThreshold){
      shooterCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
      shooterCurrentLimitsConfigs.StatorCurrentLimit = currentLimit;
      shooterCurrentLimitsConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;
      shooterCurrentLimitsConfigs.SupplyTimeThreshold = supplyTimeThreshold;

      upperConfigurator.apply(shooterCurrentLimitsConfigs);
      lowerConfigurator.apply(shooterCurrentLimitsConfigs);
    }

    @Override
    public void enableBrakeMode(boolean enable){
      upperMotorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      lowerMotorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

      upperConfigurator.apply(upperMotorOutputConfigs);
      lowerConfigurator.apply(lowerMotorOutputConfigs);
    }

    @Override
    public void updateTunableNumbers(){
      if (upperShooterKs.hasChanged(0)
        || upperShooterKv.hasChanged(0)
        || upperShooterKp.hasChanged(0)
        || upperShooterKi.hasChanged(0)
        || upperShooterKd.hasChanged(0)
        || upperShooterKa.hasChanged(0)) {
      upperMotorSlot0Configs.kS = upperShooterKs.get();
      upperMotorSlot0Configs.kV = upperShooterKv.get();
      upperMotorSlot0Configs.kP = upperShooterKp.get();
      upperMotorSlot0Configs.kI = upperShooterKi.get();
      upperMotorSlot0Configs.kD = upperShooterKd.get();
      upperMotorSlot0Configs.kA = upperShooterKa.get();

      upperConfigurator.apply(upperMotorSlot0Configs);
    }

    if (lowerShooterKs.hasChanged(0)
        || lowerShooterKv.hasChanged(0)
        || lowerShooterKp.hasChanged(0)
        || lowerShooterKi.hasChanged(0)
        || lowerShooterKd.hasChanged(0)
        || lowerShooterKa.hasChanged(0)) {
      lowerMotorSlot0Configs.kS = lowerShooterKs.get();
      lowerMotorSlot0Configs.kV = lowerShooterKv.get();
      lowerMotorSlot0Configs.kP = lowerShooterKp.get();
      lowerMotorSlot0Configs.kI = lowerShooterKi.get();
      lowerMotorSlot0Configs.kD = lowerShooterKd.get();
      lowerMotorSlot0Configs.kA = lowerShooterKa.get();

      lowerConfigurator.apply(lowerMotorSlot0Configs);
    }
  }
}
