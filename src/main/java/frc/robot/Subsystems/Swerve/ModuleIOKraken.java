package frc.robot.Subsystems.Swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants.ModuleConfig;

public class ModuleIOKraken implements ModuleIO {

    /* Hardware */
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder m_cancoder;
    private final Rotation2d absoluteEncoderOffset;


    /* Status signal */
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveSupplyCurrent;
    private final StatusSignal<Double> driveTorqueCurrent;

    private final StatusSignal<Double> turnPosition;
    private final StatusSignal<Double> turnVelocity;
    private final Supplier<Rotation2d> turnAbsolutePosition;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnSupplyCurrent;
    private final StatusSignal<Double> turnTorqueCurrent;


      /*  Controller Configs */
    private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration turnTalonConfig = new TalonFXConfiguration();

      /*  Control */
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0);

     public ModuleIOKraken(ModuleConfig config) {
    /*  Init controllers and encoders from config constants */

    driveTalon = new TalonFX(config.driveID(), "*");
    turnTalon = new TalonFX(config.turnID(), "*");
    m_cancoder = new CANcoder(config.absoluteEncoderChannel(), "*");
    absoluteEncoderOffset = config.absoluteEncoderOffset();


    /* Config motors */
    driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 50.0;
    driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -50.0;
    driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turnTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    turnTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
        turnTalonConfig.MotorOutput.Inverted =
        config.turnMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /*
    driveTalonConfig.Feedback.SensorToMechanismRatio = moduleConstants.driveReduction();
    turnTalonConfig.Feedback.SensorToMechanismRatio = moduleConstants.turnReduction();
    turnTalonConfig.ClosedLoopGeneral.ContinuousWrap = true; */

     /*  Apply configs  */
    for (int i = 0; i < 4; i++) {
      boolean error = driveTalon.getConfigurator().apply(driveTalonConfig, 0.1) == StatusCode.OK;
      error = error && (turnTalon.getConfigurator().apply(turnTalonConfig, 0.1) == StatusCode.OK);
      if (!error) break;
    }

    /* 250 hz signals */
    drivePosition = driveTalon.getPosition();
    turnPosition = turnTalon.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(250, drivePosition, turnPosition);

    /* 100 hz signals */

    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();

     turnAbsolutePosition =
        () -> Rotation2d.fromRotations(m_cancoder.getAbsolutePosition().getValueAsDouble()).minus(absoluteEncoderOffset);

    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnSupplyCurrent = turnTalon.getSupplyCurrent();
    turnTorqueCurrent = turnTalon.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
      100.0,
      driveVelocity,
      driveAppliedVolts,
      driveSupplyCurrent,
      driveTorqueCurrent,
      turnVelocity,
      turnAppliedVolts,
      turnSupplyCurrent,
      turnTorqueCurrent);

    /* Reset turn position to absolute encoder position*/
    turnTalon.setPosition(m_cancoder.getAbsolutePosition().getValueAsDouble());

    /* Optimize bus utilization*/
    driveTalon.optimizeBusUtilization(0, 1.0);
    turnTalon.optimizeBusUtilization(0, 1.0);

     }

     @Override
     public void updateInputs(ModuleIOInputs inputs){
      inputs.hasCurrentControl = true;
      inputs.driveMotorConnected = 
        BaseStatusSignal.refreshAll(
          drivePosition,
          driveVelocity,
          driveAppliedVolts,
          driveSupplyCurrent,
          driveTorqueCurrent
        ).isOK();

       inputs.turnMotorConnected =
        BaseStatusSignal.refreshAll(
          turnPosition, 
          turnVelocity, 
          turnAppliedVolts, 
          turnSupplyCurrent, 
          turnTorqueCurrent
        ).isOK();

        inputs.drivePositionRads = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
        inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

        inputs.turnAbsolutePosition = turnAbsolutePosition.get();
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVelocityRadsPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
        inputs.turnTorqueCurrentAmps = turnTorqueCurrent.getValueAsDouble();
     }

  @Override
  public void runDriveVolts(double volts){
    driveTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runTurnVolts(double volts){
    turnTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCharacterization(double input){
    driveTalon.setControl(currentControl.withOutput(input));
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {
    driveTalon.setControl(
        velocityTorqueCurrentFOC
            .withVelocity(Units.radiansToRotations(velocityRadsPerSec))
            .withFeedForward(feedForward));
  }

  @Override
  public void runTurnPositionSetpoint(double angleRads) {
    turnTalon.setControl(positionControl.withPosition(Units.radiansToRotations(angleRads)));
  }
  
  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveTalonConfig.Slot0.kP = kP;
    driveTalonConfig.Slot0.kI = kI;
    driveTalonConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveTalonConfig, 0.01);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnTalonConfig.Slot0.kP = kP;
    turnTalonConfig.Slot0.kI = kI;
    turnTalonConfig.Slot0.kD = kD;
    turnTalon.getConfigurator().apply(turnTalonConfig, 0.01);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveTalonConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnTalonConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
  }

  @Override
  public void stop() {
    driveTalon.setControl(neutralControl);
    turnTalon.setControl(neutralControl);
  }



}   