package frc.robot.Subsystems.Shooter;

import static frc.robot.Constants.Shooter.*;

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
import frc.Util.Logging.LoggedTunableNumber;

public class ShooterIOTalon implements ShooterIO {

  /* Hardware */
  private final TalonFX upperMotor = new TalonFX(0, "rio");
  private final TalonFX lowerMotor = new TalonFX(1, "rio");

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

  LoggedTunableNumber upperShooterKs = new LoggedTunableNumber("UpperShooter/ukS", ukS);
  LoggedTunableNumber upperShooterKa = new LoggedTunableNumber("UpperShooter/ukA", ukA);
  LoggedTunableNumber upperShooterKv = new LoggedTunableNumber("UpperShooter/ukV", ukV);
  LoggedTunableNumber upperShooterKp = new LoggedTunableNumber("UpperShooter/ukP", ukP);
  LoggedTunableNumber upperShooterKi = new LoggedTunableNumber("UpperShooter/ukI", ukI);
  LoggedTunableNumber upperShooterKd = new LoggedTunableNumber("UpperShooter/ukD", ukD);

  LoggedTunableNumber lowerShooterKs = new LoggedTunableNumber("LowerShooter/lkS", lkS);
  LoggedTunableNumber lowerShooterKa = new LoggedTunableNumber("LowerShooter/lkA", lkA);
  LoggedTunableNumber lowerShooterKv = new LoggedTunableNumber("LowerShooter/lkV", lkV);
  LoggedTunableNumber lowerShooterKp = new LoggedTunableNumber("LowerShooter/lkP", lkP);
  LoggedTunableNumber lowerShooterKi = new LoggedTunableNumber("LowerShooter/lkI", lkI);
  LoggedTunableNumber lowerShooterKd = new LoggedTunableNumber("LowerShooter/lkD", lkD);

  public ShooterIOTalon() {

    upperConfigurator = upperMotor.getConfigurator();
    lowerConfigurator = lowerMotor.getConfigurator();

    /* Configurations*/

    /* Current Limit Configuration */
    shooterCurrentLimitsConfigs = new CurrentLimitsConfigs();
    shooterCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    shooterCurrentLimitsConfigs.StatorCurrentLimit = 60;

    /* Motor Output */
    upperMotorOutputConfigs = new MotorOutputConfigs();
    upperMotorOutputConfigs.Inverted = UPPER_SHOOTER_INVERSION;
    upperMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    lowerMotorOutputConfigs = new MotorOutputConfigs();
    lowerMotorOutputConfigs.Inverted = LOWER_SHOOTER_INVERSION;
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
  public void updateInputs(ShooterIOInputs inputs) {
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
    inputs.upperShooterPosRad = Units.rotationsToRadians(positionUp.getValue());
    inputs.upperShooterVelocityRpm = velocityUp.getValueAsDouble() * 60;
    inputs.upperShooterAppliedVolts = upperMotor.getMotorVoltage().getValue();
    inputs.upperShooterTempCelcius = upperMotor.getDeviceTemp().getValue();
    inputs.upperShooterSetPointRpm = desiredUp;

    /* Lower inputs */
    inputs.lowerShooterPosRad = Units.rotationsToRadians(positionDown.getValue());
    inputs.lowerShooterVelocityRpm = velocityDown.getValueAsDouble() * 60;
    inputs.lowerShooterAppliedVolts = lowerMotor.getMotorVoltage().getValue();
    inputs.lowerShooterTempCelcius = lowerMotor.getDeviceTemp().getValue();
    inputs.lowerShooterSetPointRpm = desiredDown;

    inputs.shooterCurrentAmps = new double[] {supplyUp.getValue(), supplyDown.getValue()};
  }

  @Override
  public void stop() {
    upperMotor.stopMotor();
    lowerMotor.stopMotor();
  }

  @Override
  public void setVelocity(double velocityUp, double velocityDown) {
    upperVelocity.Velocity = velocityUp / 60;
    upperMotor.setControl(upperVelocity);

    lowerVelocity.Velocity = velocityDown / 60;
    lowerMotor.setControl(lowerVelocity);
  }

  @Override
  public void setVoltage(double volts) {
    upperMotor.setControl(new VoltageOut(volts).withEnableFOC(true));
    lowerMotor.setControl(new VelocityVoltage(volts).withEnableFOC(true));
  }

  @Override
  public void updateTunableNumbers() {
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
