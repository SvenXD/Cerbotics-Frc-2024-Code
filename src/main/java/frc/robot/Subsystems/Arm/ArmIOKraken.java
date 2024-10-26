package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;

public class ArmIOKraken implements ArmIO {
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;

  private final TalonFXConfiguration leftConfig;
  private final TalonFXConfiguration rightConfig;

  private final TalonFXSimState leftMotorSim;
  private final TalonFXSimState rightMotorSim;

  //  private final TalonFX leftMotorFollower;
  private static final double HOOD_LENGTH = Units.inchesToMeters(3);
  private static final double HOOD_WEIGHT = Units.lbsToKilograms(15);

  private final CANcoder armEncoder;
  private final CANcoderSimState encoderSim;

  private final StatusSignal<Double> positiionSignal;
  private final StatusSignal<Double> leftVelocitySignal;
  private final StatusSignal<Double> leftAccelSignal;
  private final StatusSignal<Double> leftErrorSignal;
  private final StatusSignal<Double> leftMotorTempSignal;
  private final StatusSignal<Double> leftMotorSupplyCurrentSignal;
  private final StatusSignal<Double> leftMotorStatorCurrentSignal;
  private final StatusSignal<Double> leftMotorSupplyVoltage;
  private final StatusSignal<Double> leftMotorVoltageSignal;

  private final StatusSignal<Double> rightVelocitySignal;
  private final StatusSignal<Double> rightAccelSignal;
  private final StatusSignal<Double> rightErrorSignal;
  private final StatusSignal<Double> rightMotorTempSignal;
  private final StatusSignal<Double> rightMotorSupplyCurrentSignal;
  private final StatusSignal<Double> rightMotorStatorCurrentSignal;
  private final StatusSignal<Double> rightMotorSupplyVoltage;
  private final StatusSignal<Double> rightMotorVoltageSignal;

  private final DynamicMotionMagicVoltage positionRequest =
      new DynamicMotionMagicVoltage(0.0, 1.1, 1.1, 100);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StaticBrake brakeRequest = new StaticBrake();

  private final SingleJointedArmSim leftSim;

  public ArmIOKraken() {

    leftMotor = new TalonFX(42, "Swerve_Canivore");
    rightMotor = new TalonFX(42, "*");

    armEncoder = new CANcoder(1, "*");
    // Load the config from the encoder so we don't overwrite the offset
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    StatusCode refreshStatus = armEncoder.getConfigurator().refresh(encoderConfig, 2.0);

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

    // Phoenix has a hardcoded offset in sim
    if (Robot.isSimulation()) {
      encoderConfig.MagnetSensor.MagnetOffset = 0.25;
    } else if (refreshStatus != StatusCode.OK || encoderConfig.MagnetSensor.MagnetOffset == 0) {
      encoderConfig.MagnetSensor.MagnetOffset =
          Preferences.getDouble("ArmleftEncoderOffset", encoderConfig.MagnetSensor.MagnetOffset);
    } else {
      Preferences.setDouble("ArmleftEncoderOffset", encoderConfig.MagnetSensor.MagnetOffset);
    }

    armEncoder.getConfigurator().apply(encoderConfig);

    leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    leftConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    leftConfig.Feedback.RotorToSensorRatio = 70;
    leftConfig.Slot0.kP = 200;
    leftConfig.Slot0.kI = 0.0;
    leftConfig.Slot0.kD = 0.0;
    leftConfig.Slot0.kS = 0.81888;

    leftConfig.Slot0.kG = 0.047416;
    leftConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(180);
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(90);
    leftConfig.MotionMagic.MotionMagicExpo_kV = 10.657;
    leftConfig.MotionMagic.MotionMagicExpo_kA = 1.2939;
    leftConfig.MotionMagic.MotionMagicCruiseVelocity = 1.1;
    leftConfig.MotionMagic.MotionMagicAcceleration = 2.1;
    leftConfig.MotionMagic.MotionMagicJerk = 100;
    leftConfig.Audio.AllowMusicDurDisable = true;
    leftConfig.ClosedLoopGeneral.ContinuousWrap = true;

    leftMotor.getConfigurator().apply(leftConfig);

    rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    rightConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    rightConfig.Feedback.RotorToSensorRatio = 70;
    rightConfig.Slot0.kP = 200;
    rightConfig.Slot0.kI = 0.0;
    rightConfig.Slot0.kD = 0.0;
    rightConfig.Slot0.kS = 0.81888;

    rightConfig.Slot0.kG = 0.047416;
    rightConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    rightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    rightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(180);
    rightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(90);
    rightConfig.MotionMagic.MotionMagicExpo_kV = 10.657;
    rightConfig.MotionMagic.MotionMagicExpo_kA = 1.2939;
    rightConfig.MotionMagic.MotionMagicCruiseVelocity = 1.1;
    rightConfig.MotionMagic.MotionMagicAcceleration = 2.1;
    rightConfig.MotionMagic.MotionMagicJerk = 100;
    rightConfig.Audio.AllowMusicDurDisable = true;
    rightConfig.ClosedLoopGeneral.ContinuousWrap = true;

    rightMotor.getConfigurator().apply(rightConfig);

    positiionSignal = armEncoder.getAbsolutePosition();
    leftVelocitySignal = armEncoder.getVelocity();
    leftAccelSignal = leftMotor.getAcceleration();
    leftErrorSignal = leftMotor.getClosedLoopError();
    leftMotorTempSignal = leftMotor.getDeviceTemp();
    leftMotorSupplyCurrentSignal = leftMotor.getSupplyCurrent();
    leftMotorStatorCurrentSignal = leftMotor.getStatorCurrent();
    leftMotorVoltageSignal = leftMotor.getMotorVoltage();
    leftMotorSupplyVoltage = leftMotor.getSupplyVoltage();

    rightVelocitySignal = armEncoder.getVelocity();
    rightAccelSignal = rightMotor.getAcceleration();
    rightErrorSignal = rightMotor.getClosedLoopError();
    rightMotorTempSignal = rightMotor.getDeviceTemp();
    rightMotorSupplyCurrentSignal = rightMotor.getSupplyCurrent();
    rightMotorStatorCurrentSignal = rightMotor.getStatorCurrent();
    rightMotorVoltageSignal = rightMotor.getMotorVoltage();
    rightMotorSupplyVoltage = rightMotor.getSupplyVoltage();

    leftSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(2),
            100,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), HOOD_WEIGHT),
            HOOD_LENGTH,
            Units.degreesToRadians(180),
            Units.degreesToRadians(90),
            true,
            Units.degreesToRadians(100));

    leftMotorSim = leftMotor.getSimState();
    rightMotorSim = rightMotor.getSimState();
    encoderSim = armEncoder.getSimState();
  }

  /**
   * Update the inputs for the arm left
   *
   * @param inputs The inputs to update
   */
  @Override
  public void updateInputs(ArmInputs inputs) {
    if (Robot.isSimulation()) {
      leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      double leftVoltage = leftMotorSim.getMotorVoltage();

      leftSim.setInput(leftVoltage);
      leftSim.update(0.02);

      double leftPosRot = Units.radiansToRotations(leftSim.getAngleRads());
      double leftVelRot = Units.radiansToRotations(leftSim.getVelocityRadPerSec());

      encoderSim.setRawPosition(leftPosRot);
      encoderSim.setVelocity(leftVelRot);
    }

    double leftRot = positiionSignal.getValue();

    inputs.angleDegrees = Units.rotationsToDegrees(leftRot);
    inputs.closedLoopError = Units.rotationsToDegrees(leftErrorSignal.getValue());

    inputs.leftVelocityDps = Units.rotationsToDegrees(leftVelocitySignal.getValue());
    inputs.leftAccelDpsSq = Units.rotationsToDegrees(leftAccelSignal.getValue());
    inputs.leftMotorTemp = leftMotorTempSignal.getValue();
    inputs.leftMotorSupplyCurrent = leftMotorSupplyCurrentSignal.getValue();
    inputs.leftMotorStatorCurrent = leftMotorStatorCurrentSignal.getValue();
    inputs.leftMotorVoltage = leftMotorVoltageSignal.getValue();
    inputs.leftMotorSupplyVoltage = leftMotorSupplyVoltage.getValue();

    inputs.rightVelocityDps = Units.rotationsToDegrees(rightVelocitySignal.getValue());
    inputs.rightAccelDpsSq = Units.rotationsToDegrees(rightAccelSignal.getValue());
    inputs.rightMotorTemp = rightMotorTempSignal.getValue();
    inputs.rightMotorSupplyCurrent = rightMotorSupplyCurrentSignal.getValue();
    inputs.rightMotorStatorCurrent = rightMotorStatorCurrentSignal.getValue();
    inputs.rightMotorVoltage = rightMotorVoltageSignal.getValue();
    inputs.rightMotorSupplyVoltage = rightMotorSupplyVoltage.getValue();

    BaseStatusSignal.refreshAll(
        positiionSignal,
        leftVelocitySignal,
        leftAccelSignal,
        leftErrorSignal,
        leftMotorTempSignal,
        leftMotorSupplyCurrentSignal,
        leftMotorStatorCurrentSignal,
        leftMotorVoltageSignal,
        leftMotorSupplyVoltage,
        rightVelocitySignal,
        rightAccelSignal,
        rightErrorSignal,
        rightMotorTempSignal,
        rightMotorSupplyCurrentSignal,
        rightMotorStatorCurrentSignal,
        rightMotorVoltageSignal,
        rightMotorSupplyVoltage);
  }

  /**
   * Set the target angle for the arm
   *
   * @param angle Target arm angle
   */
  @Override
  public void setTargetAngle(Rotation2d angle) {
    leftMotor.setControl(
        positionRequest.withPosition(angle.getRotations()).withVelocity(1.1).withAcceleration(2.1));
    rightMotor.setControl(
        positionRequest.withPosition(angle.getRotations()).withVelocity(1.1).withAcceleration(2.1));
  }

  @Override
  public void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel) {
    leftMotor.setControl(
        positionRequest
            .withPosition(angle.getRotations())
            .withVelocity(maxVel)
            .withAcceleration(maxAccel));
  }

  /**
   * Set the voltage output to the arm left motors
   *
   * @param volts Voltage to output
   */
  @Override
  public void setVoltage(double volts) {
    leftMotor.setControl(voltageRequest.withOutput(volts));
    rightMotor.setControl(voltageRequest.withOutput(volts));
  }
}
