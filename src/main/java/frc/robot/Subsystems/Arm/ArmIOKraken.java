package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.StatusCode;
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

  private final DynamicMotionMagicVoltage positionRequest =
      new DynamicMotionMagicVoltage(0.0, 1.1, 1.1, 100);

  private final MotionMagicVoltage motionMagicTest = new MotionMagicVoltage(0);
  private final DutyCycleOut voltageRequest = new DutyCycleOut(0.0);
  private final StaticBrake brakeRequest = new StaticBrake();

  private final SingleJointedArmSim leftSim;

  public ArmIOKraken() {

    leftMotor = new TalonFX(14, "Swerve_Canivore");
    rightMotor = new TalonFX(41, "rio");

    armEncoder = new CANcoder(16, "Swerve_Canivore");
    // Load the config from the encoder so we don't overwrite the offset
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    StatusCode refreshStatus = armEncoder.getConfigurator().refresh(encoderConfig, 2.0);

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

    // Phoenix has a hardcoded offset in sim
    if (Robot.isSimulation()) {
      encoderConfig.MagnetSensor.MagnetOffset = 0.25;
    }

    armEncoder.getConfigurator().apply(encoderConfig);

    leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    leftMotor.getConfigurator().apply(leftConfig);

    rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightMotor.getConfigurator().apply(rightConfig);

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

    inputs.angleDegrees = getAngleOfArm();

    /*inputs.leftVelocityDps = Units.rotationsToDegrees(leftVelocitySignal.getValue());
    inputs.leftAccelDpsSq = Units.rotationsToDegrees(leftAccelSignal.getValue());
    inputs.leftMotorTemp = leftMotorTempSignal.getValue();
    inputs.leftMotorSupplyCurrent = leftMotorSupplyCurrentSignal.getValue();
    inputs.leftMotorStatorCurrent = leftMotorStatorCurrentSignal.getValue();
    inputs.leftMotorVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftMotorSupplyVoltage = leftMotorSupplyVoltage.getValue();

    inputs.rightVelocityDps = Units.rotationsToDegrees(rightVelocitySignal.getValue());
    inputs.rightAccelDpsSq = Units.rotationsToDegrees(rightAccelSignal.getValue());
    inputs.rightMotorTemp = rightMotorTempSignal.getValue();
    inputs.rightMotorSupplyCurrent = rightMotorSupplyCurrentSignal.getValue();
    inputs.rightMotorStatorCurrent = rightMotorStatorCurrentSignal.getValue();
    inputs.rightMotorVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightMotorSupplyVoltage = rightMotorSupplyVoltage.getValue();

    /*BaseStatusSignal.refreshAll(
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
    rightMotorSupplyVoltage);*/
  }

  public double getAngleOfArm() {
    return Units.rotationsToDegrees(armEncoder.getAbsolutePosition().getValueAsDouble());
  }

  /**
   * Set the target angle for the arm
   *
   * @param angle Target arm angle
   */
  @Override
  public void setTargetAngle(Rotation2d angle) {
    rightMotor.setControl(motionMagicTest.withPosition(angle.getRotations()).withSlot(0));
    /*rightMotor.setControl(
    positionRequest.withPosition(angle.getRotations()).withVelocity(1.1).withAcceleration(2.1));*/
  }

  @Override
  public void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel) {
    /*leftMotor.setControl(
    positionRequest
        .withPosition(angle.getRotations())
        .withVelocity(maxVel)
        .withAcceleration(maxAccel));*/
  }

  /**
   * Set the voltage output to the arm left motors
   *
   * @param volts Voltage to output
   */
  @Override
  public void setVoltage(double volts, double feedforward) {
    leftMotor.set(volts + feedforward);
    rightMotor.set(volts + feedforward);
  }

  @Override
  public void enableBreak(boolean enable) {
    rightConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    if (rightConfig.MotorOutput.NeutralMode == leftConfig.MotorOutput.NeutralMode) {

    } else {
      leftConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

      leftMotor.getConfigurator().apply(leftConfig);
      rightMotor.getConfigurator().apply(rightConfig);
    }
  }
}
