package frc.robot.Subsystems.JointRanger;

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

public class ArmIOTest implements RangerIO {
  private final TalonFX jointMotor;
  private final TalonFXSimState jointMotorSim;

  //  private final TalonFX jointMotorFollower;
  private static final double HOOD_LENGTH = Units.inchesToMeters(3);
  private static final double HOOD_WEIGHT = Units.lbsToKilograms(15);

  private final CANcoder jointEncoder;
  private final CANcoderSimState encoderSim;

  private final StatusSignal<Double> jointPositiionSignal;
  private final StatusSignal<Double> jointVelocitySignal;
  private final StatusSignal<Double> jointAccelSignal;
  private final StatusSignal<Double> jointErrorSignal;
  private final StatusSignal<Double> jointMotorTempSignal;
  private final StatusSignal<Double> jointMotorSupplyCurrentSignal;
  private final StatusSignal<Double> jointMotorStatorCurrentSignal;

  private final StatusSignal<Double> jointMotorSupplyVoltage;
  private final StatusSignal<Double> jointMotorVoltageSignal;
  //  private final StatusSignal<Double> jointFollowerTempSignal;
  //  private final StatusSignal<Double> jointFollowerSupplyCurrentSignal;
  //  private final StatusSignal<Double> jointFollowerStatorCurrentSignal;
  //  private final StatusSignal<Double> jointFollowerVoltageSignal;

  private final DynamicMotionMagicVoltage positionRequest =
      new DynamicMotionMagicVoltage(0.0, 1.1, 1.1, 100);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StaticBrake brakeRequest = new StaticBrake();

  private final SingleJointedArmSim jointSim;

  public ArmIOTest() {
    jointMotor = new TalonFX(1, "*");
    jointEncoder = new CANcoder(1, "*");
    //    jointMotorFollower = new TalonFX(Constants.ArmJoint.followerMotorID,
    // Constants.canivoreBusName);

    // Load the config from the encoder so we don't overwrite the offset
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    StatusCode refreshStatus = jointEncoder.getConfigurator().refresh(encoderConfig, 2.0);

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

    // Phoenix has a hardcoded offset in sim
    if (Robot.isSimulation()) {
      encoderConfig.MagnetSensor.MagnetOffset = 0.25;
    } else if (refreshStatus != StatusCode.OK || encoderConfig.MagnetSensor.MagnetOffset == 0) {
      encoderConfig.MagnetSensor.MagnetOffset =
          Preferences.getDouble("ArmJointEncoderOffset", encoderConfig.MagnetSensor.MagnetOffset);
    } else {
      Preferences.setDouble("ArmJointEncoderOffset", encoderConfig.MagnetSensor.MagnetOffset);
    }

    jointEncoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = jointEncoder.getDeviceID();
    motorConfig.Feedback.RotorToSensorRatio = 70;
    motorConfig.Slot0.kP = 200;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;
    //    motorConfig.Slot0.kV = Constants.ArmJoint.kV;
    //    motorConfig.Slot0.kA = Constants.ArmJoint.kA;
    motorConfig.Slot0.kS = 0.81888;
    ;
    motorConfig.Slot0.kG = 0.047416;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(180);
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(90);
    motorConfig.MotionMagic.MotionMagicExpo_kV = 10.657;
    motorConfig.MotionMagic.MotionMagicExpo_kA = 1.2939;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 1.1;
    motorConfig.MotionMagic.MotionMagicAcceleration = 2.1;
    motorConfig.MotionMagic.MotionMagicJerk = 100;
    motorConfig.Audio.AllowMusicDurDisable = true;
    motorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    jointMotor.getConfigurator().apply(motorConfig);
    //    jointMotorFollower.getConfigurator().apply(motorConfig);

    jointPositiionSignal = jointEncoder.getAbsolutePosition();
    jointVelocitySignal = jointEncoder.getVelocity();
    jointAccelSignal = jointMotor.getAcceleration();
    jointErrorSignal = jointMotor.getClosedLoopError();
    jointMotorTempSignal = jointMotor.getDeviceTemp();
    jointMotorSupplyCurrentSignal = jointMotor.getSupplyCurrent();
    jointMotorStatorCurrentSignal = jointMotor.getStatorCurrent();
    jointMotorVoltageSignal = jointMotor.getMotorVoltage();
    jointMotorSupplyVoltage = jointMotor.getSupplyVoltage();
    //    jointFollowerTempSignal = jointMotorFollower.getDeviceTemp();
    //    jointFollowerSupplyCurrentSignal = jointMotorFollower.getSupplyCurrent();
    //    jointFollowerStatorCurrentSignal = jointMotorFollower.getStatorCurrent();
    //    jointFollowerVoltageSignal = jointMotorFollower.getMotorVoltage();

    jointSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(2),
            100,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), HOOD_WEIGHT),
            HOOD_LENGTH,
            Units.degreesToRadians(180),
            Units.degreesToRadians(90),
            true,
            Units.degreesToRadians(100));

    jointMotorSim = jointMotor.getSimState();
    encoderSim = jointEncoder.getSimState();

    //    jointMotorFollower.setControl(new Follower(jointMotor.getDeviceID(), false));
  }

  /**
   * Update the inputs for the arm joint
   *
   * @param inputs The inputs to update
   */
  @Override
  public void updateInputs(ArmJointInputs inputs) {
    if (Robot.isSimulation()) {
      jointMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      double jointVoltage = jointMotorSim.getMotorVoltage();

      jointSim.setInput(jointVoltage);
      jointSim.update(0.02);

      double jointPosRot = Units.radiansToRotations(jointSim.getAngleRads());
      double jointVelRot = Units.radiansToRotations(jointSim.getVelocityRadPerSec());

      encoderSim.setRawPosition(jointPosRot);
      encoderSim.setVelocity(jointVelRot);
    }

    double jointRot = jointPositiionSignal.getValue();

    inputs.jointAngleDegrees = Units.rotationsToDegrees(jointRot);
    inputs.jointVelocityDps = Units.rotationsToDegrees(jointVelocitySignal.getValue());
    inputs.jointAccelDpsSq = Units.rotationsToDegrees(jointAccelSignal.getValue());
    inputs.jointClosedLoopError = Units.rotationsToDegrees(jointErrorSignal.getValue());
    inputs.jointMotorTemp = jointMotorTempSignal.getValue();
    inputs.jointMotorSupplyCurrent = jointMotorSupplyCurrentSignal.getValue();
    inputs.jointMotorStatorCurrent = jointMotorStatorCurrentSignal.getValue();
    inputs.jointMotorVoltage = jointMotorVoltageSignal.getValue();
    inputs.jointMotorSupplyVoltage = jointMotorSupplyVoltage.getValue();
    //    inputs.jointFollowerTemp = jointFollowerTempSignal.getValue();
    //    inputs.jointFollowerSupplyCurrent = jointFollowerSupplyCurrentSignal.getValue();
    //    inputs.jointFollowerStatorCurrent = jointFollowerStatorCurrentSignal.getValue();
    //    inputs.jointFollowerVoltage = jointFollowerVoltageSignal.getValue();

    BaseStatusSignal.refreshAll(
        jointPositiionSignal,
        jointVelocitySignal,
        jointAccelSignal,
        jointErrorSignal,
        jointMotorTempSignal,
        jointMotorSupplyCurrentSignal,
        jointMotorStatorCurrentSignal,
        jointMotorVoltageSignal,
        jointMotorSupplyVoltage);
  }

  /**
   * Set the target angle for the arm joint
   *
   * @param angle Target arm angle
   */
  @Override
  public void setTargetAngle(Rotation2d angle) {
    jointMotor.setControl(
        positionRequest.withPosition(angle.getRotations()).withVelocity(1.1).withAcceleration(2.1));
  }

  @Override
  public void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel) {
    jointMotor.setControl(
        positionRequest
            .withPosition(angle.getRotations())
            .withVelocity(maxVel)
            .withAcceleration(maxAccel));
  }

  /**
   * Set the voltage output to the arm joint motors
   *
   * @param volts Voltage to output
   */
  @Override
  public void setVoltage(double volts) {
    jointMotor.setControl(voltageRequest.withOutput(volts));
  }
}
