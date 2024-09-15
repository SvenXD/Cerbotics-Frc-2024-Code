package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.Inches;



public class ElevatorIOTalon implements ElevatorIO{

  // Val of dg input / motors
  private ElevatorSim elevatorSim =
      new ElevatorSim(
          12.0 / 96.6,
          12.0 / 37.0,
          DCMotor.getKrakenX60Foc(1),
          0.0,
          Units.inchesToMeters(13.6),
          true,
          0.0);

  // From what i understand status signal is just better ig
  private double startingTopRollerPos;

  private static final double GEAR_RATION = 3.75;
  private static final double IN_PER_ROT = Math.PI * 1 / GEAR_RATION;
  private static final double UPTAKE_ROTATIONS = 18;
  public final DigitalInput elevatorBeamBreak = new DigitalInput(1);
  public final StatusSignal<Double> elevatorTopRollerDCSignal;
  public final StatusSignal<Double> elevatorExtensionSignal;
  public final StatusSignal<Double> elevatorMiddleRollerDCSignal;
  public final StatusSignal<Double> elevatorTopRollerCurrentStatorSignal;
  public final StatusSignal<Double> elevatorTopRollerCurrentSupplySignal;
  public final StatusSignal<Double> elevatorTopRollerPosSignal;

  /* Hardware */
  private final TalonFX elevatorTop = new TalonFX(0, "*");
  private final TalonFX elevatorExtension = new TalonFX(1, "*");
  private final TalonFX elevatorMiddleRoller = new TalonFX(2, "*");

  public final MotionMagicExpoVoltage elevatorExtensionRequest =
      new MotionMagicExpoVoltage(0).withEnableFOC(true);
  public final MotionMagicVoltage elevatorTopRollerRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);
  public final VoltageOut elevatorMiddleRollerRequest = new VoltageOut(0);
  public final DutyCycleOut elevatorShooterRequest = new DutyCycleOut(0);

  public ElevatorIOTalon() {

    var ramp = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0.005);

    elevatorTopRollerDCSignal = elevatorTop.getDutyCycle();
    elevatorMiddleRollerDCSignal = elevatorMiddleRoller.getDutyCycle();
    elevatorExtensionSignal = elevatorExtension.getPosition();
    elevatorTopRollerCurrentStatorSignal = elevatorTop.getStatorCurrent();
    elevatorTopRollerCurrentSupplySignal = elevatorTop.getSupplyCurrent();
    elevatorTopRollerPosSignal = elevatorTop.getPosition();

    startingTopRollerPos = elevatorTopRollerPosSignal.getValue();

    var extensionConfig = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    var motionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(80)
            .withMotionMagicAcceleration(600) // todo 2225.530583
            .withMotionMagicJerk(1200)
            .withMotionMagicExpo_kV(0.154)
            .withMotionMagicExpo_kA(0.012);

    var rollerConfig =
        new MotionMagicConfigs().withMotionMagicCruiseVelocity(90).withMotionMagicAcceleration(800);
    var rollerslot0Condig = new Slot0Configs().withKP(10).withKV(0.2);
    var slot0Config =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0.15) // guesstimate
            .withKP(5) // guess
            .withKV(12.0 / 96.6 * 0.8); // guesstimate

    var curLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(60);
    var rollercurLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(90);
    var softLimitConfig =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold((17.6) / IN_PER_ROT)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0);

    TalonFXConfiguration elevatorExtensionConfig =
        new TalonFXConfiguration()
            .withSlot0(slot0Config)
            .withMotionMagic(motionMagicConfig)
            .withMotorOutput(extensionConfig)
            .withCurrentLimits(curLimits)
            .withSoftwareLimitSwitch(softLimitConfig);

    elevatorExtension.setControl(elevatorExtensionRequest);

    /*elevatorTab.addNumber("elevator Shooter DC", elevatorTopRollerDCSignal::getValue);
    elevatorTab.addNumber("elevator Extension Position", this::getExtensionInches);
    elevatorTab.addNumber("elevator Top Stator", elevatorTopRollerCurrentStatorSignal::getValue);
    elevatorTab.addNumber("elevator Top Supply", elevatorTopRollerCurrentSupplySignal::getValue);
    elevatorTab.addNumber("elevator Top Position", elevatorTopRollerPosSignal::getValue);
    elevatorTab.addNumber("elevator Servo Postition", trapServo::getPosition);
    elevatorTab.addNumber("Top roller desired Postition", () -> startingTopRollerPos + UPTAKE_ROTATIONS);*/

    elevatorExtension.setPosition(0);
    elevatorExtension.setControl(elevatorExtensionRequest.withPosition(0));

    var middleConfig =
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);

    TalonFXConfiguration elevatorMiddleRollerConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(curLimits)
            .withMotorOutput(middleConfig)
            .withOpenLoopRamps(ramp);
    TalonFXConfiguration topRollerCfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
            .withOpenLoopRamps(ramp) // todo remove this?
            .withMotionMagic(rollerConfig)
            .withSlot0(rollerslot0Condig)
            .withCurrentLimits(rollercurLimits);

    elevatorMiddleRoller.setControl(elevatorMiddleRollerRequest);
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        elevatorTopRollerDCSignal,
        elevatorExtensionSignal,
        elevatorMiddleRollerDCSignal,
        elevatorTopRollerCurrentStatorSignal,
        elevatorTopRollerCurrentStatorSignal,
        elevatorTopRollerPosSignal);

    elevatorExtension.getConfigurator().apply(elevatorExtensionConfig);
    elevatorMiddleRoller.getConfigurator().apply(elevatorMiddleRollerConfig);
    elevatorTop.getConfigurator().apply(topRollerCfg);
    
    elevatorTop.optimizeBusUtilization();
    elevatorExtension.optimizeBusUtilization();
    elevatorMiddleRoller.optimizeBusUtilization();
  }

  public void setelevatorMiddleDutyCycle(double speed) {
    elevatorMiddleRoller.setControl(elevatorMiddleRollerRequest.withOutput(speed));
  }

  public void setelevatorTopDutyCycle(double speed) {
    elevatorTop.setControl(elevatorShooterRequest.withOutput(speed));
  }

  Follower follower = new Follower(1, false);  //TODO: TOP MOTOR

  public void setTopRollerPosition() {
    elevatorTop.setControl(elevatorTopRollerRequest.withPosition(startingTopRollerPos + UPTAKE_ROTATIONS));
    elevatorMiddleRoller.setControl(follower);
  }

  public void setelevatorExtensionMotor(Measure<Distance> position) {
      elevatorExtension.setControl(elevatorExtensionRequest.withPosition(position.in(Inches) / IN_PER_ROT));
    }

  @Override
  public void superPeriodic(){
    BaseStatusSignal.refreshAll(
      elevatorTopRollerDCSignal,
      elevatorExtensionSignal,
      elevatorMiddleRollerDCSignal,
      elevatorTopRollerCurrentStatorSignal,
      elevatorTopRollerCurrentSupplySignal,
      elevatorTopRollerPosSignal);
  }
}
