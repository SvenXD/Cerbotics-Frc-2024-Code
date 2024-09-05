package frc.robot.Subsystems.Pivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
  private final TalonFX pivotMotor;
  private double pivotSetpoint;
  private final double gearRatio = 75;
  private double latencyCompensatedPosition = 0;
  public final StatusSignal<Double> pivotPositionSignal;
  public final StatusSignal<Double> pivotSetpointSignal;
  public final StatusSignal<Double> pivotVelocitySignal;

  private MotionMagicVoltage pivotPositionVoltage = new MotionMagicVoltage(0).withEnableFOC(true);
  private VoltageOut openloop = new VoltageOut(0);
  private boolean isOpenLoop = true;

  private static final double HOOD_LENGTH = Units.inchesToMeters(3);
  private static final double HOOD_WEIGHT = Units.lbsToKilograms(15);

  private SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          gearRatio,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), HOOD_WEIGHT),
          HOOD_LENGTH,
          Units.degreesToRadians(0),
          Units.degreesToRadians(68),
          true,
          Units.degreesToRadians(0));

  private TalonFXSimState pivotSimState;

  public PivotIOSim() {
    MotorOutputConfigs outputConfigs =
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
    var curLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80);

    Slot0Configs voltageConfigs =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKP(165) // was 165.,
            .withKI(0)
            .withKD(4);

    FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(gearRatio);
    var velo = DegreesPerSecond.of(180);

    MotionMagicConfigs magicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(velo.in(RotationsPerSecond))
            .withMotionMagicAcceleration(
                velo.in(RotationsPerSecond) * 100) // Reach max vel in 1 second
            .withMotionMagicJerk(velo.in(RotationsPerSecond) * 100 * 10);
    var feederOutput = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    var softLimitConfig =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(65)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0);

    TalonFXConfiguration pivotMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(outputConfigs)
            .withSlot0(voltageConfigs)
            .withMotionMagic(magicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(feederOutput)
            .withCurrentLimits(curLimits)
            .withSoftwareLimitSwitch(softLimitConfig);

    pivotMotor = new TalonFX(0, "*");
    pivotMotor.getConfigurator().apply(pivotMotorConfig);

    pivotPositionSignal = pivotMotor.getPosition();
    pivotSetpointSignal = pivotMotor.getClosedLoopReference();
    pivotVelocitySignal = pivotMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, pivotPositionSignal, pivotSetpointSignal, pivotVelocitySignal);

    pivotMotor.optimizeBusUtilization();
    pivotSimState = pivotMotor.getSimState();
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRotations(latencyCompensatedPosition);
  }

  @Override
  public void setOpenLoop(double v) {
    isOpenLoop = true;
    openloop = openloop.withOutput(v);
  }

  @Override
  public void setDesiredAngle(Measure<Angle> angle) {
    pivotSetpoint = angle.in(Rotations);
    isOpenLoop = false;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotPositionDeg = Units.radiansToDegrees(pivotSim.getAngleRads());
    inputs.pivotVoltage = pivotSimState.getMotorVoltage();
  }

  @Override
  public void superSimPeriodic() {
    pivotSimState = pivotMotor.getSimState();
    pivotSimState.setSupplyVoltage(12);
    pivotSimState.Orientation = ChassisReference.Clockwise_Positive;
    pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
    pivotSim.update(0.02);
    pivotSimState.setRawRotorPosition(
        Units.radiansToRotations(pivotSim.getAngleRads()) * gearRatio);
    pivotSimState.setRotorVelocity(
        Units.radiansToRotations(pivotSim.getVelocityRadPerSec()) * gearRatio);

    if (!isOpenLoop) {
      pivotMotor.setControl(
          pivotPositionVoltage.withPosition(pivotSetpoint).withOverrideBrakeDurNeutral(true));
    } else {
      pivotMotor.setControl(openloop.withOverrideBrakeDurNeutral(true));
    }
    latencyCompensatedPosition =
        BaseStatusSignal.getLatencyCompensatedValue(pivotPositionSignal, pivotVelocitySignal);
  }
}
