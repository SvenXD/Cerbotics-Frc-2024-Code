package frc.robot.Subsystems.Arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.Util.Logging.LoggedTunableNumber;
import frc.robot.Robot;

public class ArmIOKraken implements ArmIO {

  /* Hardware */
  private final TalonFX leftArmMotor;
  private final TalonFX rightArmMotor;

  /* Configurators */
  private static MotorOutputConfigs leftMotorConfig;
  private static TalonFXConfiguration leftConfig;

  private static MotorOutputConfigs rightMotorConfig;
  private static TalonFXConfiguration rightConfig;

  /* Configs */
  private final CurrentLimitsConfigs currentLimitsConfigs;
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;

  public final StatusSignal<Double> armPositionSignal;
  public final StatusSignal<Double> armSetpointSignal;
  public final StatusSignal<Double> armVelocitySignal;

  /* Ranger stuff */
  private final CANcoder jointEncoder;
  private final CANcoderSimState encoderSim;

  private final StatusSignal<Double> jointPositiionSignal;
  private final StatusSignal<Double> jointVelocitySignal;
  private final StatusSignal<Double> jointAccelSignal;
  private final StatusSignal<Double> jointErrorSignal;
  private final StatusSignal<Double> jointMotorTempSignal;
  private final StatusSignal<Double> jointMotorSupplyCurrentSignal;
  private final StatusSignal<Double> jointMotorStatorCurrentSignal;
  /* Gains */
  LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.0);
  LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.3);
  LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.12);
  LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 10.0);
  LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0);

  private MotionMagicVoltage pivotPositionVoltage = new MotionMagicVoltage(0).withEnableFOC(true);
  private VoltageOut openloop = new VoltageOut(0);
  private boolean isOpenLoop = true;
  private static double armsetPoint = 0.0;

  private static final double HOOD_LENGTH = Units.inchesToMeters(3);
  private static final double HOOD_WEIGHT = Units.lbsToKilograms(15);

  private SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          100,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), HOOD_WEIGHT),
          HOOD_LENGTH,
          Units.degreesToRadians(90),
          Units.degreesToRadians(190),
          false,
          Units.degreesToRadians(92));

  private TalonFXSimState pivotSimState;

  public ArmIOKraken() {

    jointEncoder = new CANcoder(1, "*");

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    leftMotorConfig =
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
    rightMotorConfig = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    currentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80);

    slot0Configs = new Slot0Configs();
    slot0Configs.kA = kA.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();

    FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs()
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(70); // TODO ASK FOR GEAR RATIO
    var velo = DegreesPerSecond.of(180);

    motionMagicConfigs =
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

    leftConfig =
        new TalonFXConfiguration()
            .withMotorOutput(leftMotorConfig)
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(feederOutput)
            .withCurrentLimits(currentLimitsConfigs)
            .withSoftwareLimitSwitch(softLimitConfig);

    rightConfig =
        new TalonFXConfiguration()
            .withMotorOutput(rightMotorConfig)
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(feederOutput)
            .withCurrentLimits(currentLimitsConfigs)
            .withSoftwareLimitSwitch(softLimitConfig);

    leftArmMotor = new TalonFX(0, "*");
    leftArmMotor.setControl(new StaticBrake());
    leftArmMotor.getConfigurator().apply(leftConfig);

    rightArmMotor = new TalonFX(1, "*");
    rightArmMotor.setControl(new StaticBrake());
    rightArmMotor.getConfigurator().apply(rightConfig);

    armPositionSignal = leftArmMotor.getPosition();
    armSetpointSignal = leftArmMotor.getClosedLoopReference();
    armVelocitySignal = leftArmMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, armPositionSignal, armSetpointSignal, armVelocitySignal);

    leftArmMotor.optimizeBusUtilization();

    pivotSimState = leftArmMotor.getSimState();

    if (Robot.isSimulation()) {
      encoderConfig.MagnetSensor.MagnetOffset = 0.25;
    }

    jointEncoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = jointEncoder.getDeviceID();

    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    motorConfig.Audio.AllowMusicDurDisable = true;
    motorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    //    jointMotorFollower.getConfigurator().apply(motorConfig);

    jointPositiionSignal = jointEncoder.getAbsolutePosition();
    jointVelocitySignal = jointEncoder.getVelocity();
    jointAccelSignal = leftArmMotor.getAcceleration();
    jointErrorSignal = leftArmMotor.getClosedLoopError();
    jointMotorTempSignal = leftArmMotor.getDeviceTemp();
    jointMotorSupplyCurrentSignal = leftArmMotor.getSupplyCurrent();
    jointMotorStatorCurrentSignal = leftArmMotor.getStatorCurrent();
    //    jointFollowerTempSignal = jointMotorFollower.getDeviceTemp();
    //    jointFollowerSupplyCurrentSignal = jointMotorFollower.getSupplyCurrent();
    //    jointFollowerStatorCurrentSignal = jointMotorFollower.getStatorCurrent();
    //    jointFollowerVoltageSignal = jointMotorFollower.getMotorVoltage();
    encoderSim = jointEncoder.getSimState();
  }

  @Override
  public void setOpenLoop(double v) {
    isOpenLoop = true;
    leftArmMotor.setControl(openloop.withOutput(v));
  }

  @Override
  public void setDesiredAngle(Rotation2d angle) {
    leftArmMotor.setControl(pivotPositionVoltage.withPosition(angle.getRotations()));
    isOpenLoop = false;
  }

  @Override
  public void updateInputs(ArmIoInputs inputs) {
    inputs.pivotVel = armVelocitySignal.getValue();
    inputs.setPoint = armsetPoint;
    inputs.pivotVoltage = pivotSim.getCurrentDrawAmps();

    inputs.currentAngle = Units.rotationsToDegrees(jointPositiionSignal.getValueAsDouble());

    BaseStatusSignal.refreshAll(armPositionSignal, armSetpointSignal, armVelocitySignal);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    leftMotorConfig.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    rightMotorConfig.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    leftArmMotor.getConfigurator().apply(leftMotorConfig);
    rightArmMotor.getConfigurator().apply(rightMotorConfig);
  }

  @Override
  public void updateTunableNumbers() {
    if (kA.hasChanged(0)
        || kS.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)) {
      slot0Configs.kA = kA.get();
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();

      leftArmMotor.getConfigurator().apply(slot0Configs);
      rightArmMotor.getConfigurator().apply(slot0Configs);
    }
  }

  @Override
  public void superSimPeriodic() {
    pivotSimState = leftArmMotor.getSimState();
    pivotSimState.setSupplyVoltage(12);
    pivotSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
    pivotSim.update(0.02);
    pivotSimState.setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads()) * 100);
    pivotSimState.setRotorVelocity(Units.radiansToRotations(pivotSim.getVelocityRadPerSec()) * 100);
    BaseStatusSignal.refreshAll(
        jointPositiionSignal,
        jointVelocitySignal,
        jointAccelSignal,
        jointErrorSignal,
        jointMotorTempSignal,
        jointMotorSupplyCurrentSignal,
        jointMotorStatorCurrentSignal);
  }
}
