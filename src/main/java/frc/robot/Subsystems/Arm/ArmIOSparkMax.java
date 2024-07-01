package frc.robot.Subsystems.Arm;

import com.revrobotics.CANSparkMax;

import static frc.robot.Constants.Arm.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.Util.LoggedTunableNumber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ArmIOSparkMax implements ArmIO{

    /* Hardware */
  private final CANSparkMax leftMotor = new CANSparkMax(LEFT_ARM_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(RIGHT_ARM_ID, MotorType.kBrushless);

  private final CANcoder m_encoder = new CANcoder(ABSOLUTE_ENCODER_ID, "Swerve_Canivore");

  /* PID Gains */

  LoggedTunableNumber armKg = new LoggedTunableNumber("ArmPID/kG", kG);
  LoggedTunableNumber armKs = new LoggedTunableNumber("ArmPID/kS", kS);
  LoggedTunableNumber armKa = new LoggedTunableNumber("ArmPID/kA", kA);
  LoggedTunableNumber armKv = new LoggedTunableNumber("ArmPID/kV", kV);
  LoggedTunableNumber armKp = new LoggedTunableNumber("ArmPID/kP", kP);
  LoggedTunableNumber armKi = new LoggedTunableNumber("ArmPID/kI", kI);
  LoggedTunableNumber armKd = new LoggedTunableNumber("ArmPID/kD", kD);


    private final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(kMaxVelocityRadPerSecond, kMaxAccelerationMetersPerSecondSquared);

    private ProfiledPIDController m_controller =
    new ProfiledPIDController(armKp.get(), armKi.get(), armKd.get(), m_constraints, kPeriod);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(armKs.get(), armKg.get(), armKv.get(), armKa.get());
        
    private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    public ArmIOSparkMax(){

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    encoderConfig.MagnetSensor.MagnetOffset = 0.0;

    m_encoder.getPosition().setUpdateFrequency(100);
    m_encoder.getConfigurator().apply(encoderConfig);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
    
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
    
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);
    
        leftMotor.setCANTimeout(0);
        rightMotor.setCANTimeout(0);
    
        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setIdleMode(IdleMode.kBrake);

    }


  public double getArmAngle() {            
    return (m_encoder.getAbsolutePosition().getValueAsDouble() * 360)  + 51.6;
  }

  public ProfiledPIDController getController() {
    return m_controller;
  }

  public void setVoltage(double output, State setpoint){
    double feedfoward =  m_feedforward.calculate(setpoint.position, setpoint.velocity);
    rightMotor.setVoltage(output + feedfoward);
    leftMotor.setVoltage(output + feedfoward);
  }
  
  @Override
  public void updateInputs(ArmIoInputs inputs){
    inputs.leftAppliedVolts = leftMotor.getBusVoltage();
    inputs.rightAppliedVolts = rightMotor.getBusVoltage();
    inputs.leftTempCelcius = leftMotor.getMotorTemperature();
    inputs.rightTempCelcius = rightMotor.getMotorTemperature();

    inputs.currentAngle = getArmAngle();
    inputs.setPoint = getController().getGoal().position;
  }

  @Override
  public void putThisInPeriodicBecauseOtherwiseItWontWorkAndItsReallyImportant(){
    setVoltage(m_controller.calculate(getArmAngle()),m_controller.getSetpoint());
  }

  @Override
  public void setBrakeMode(){
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void setCoastMode(){
    rightMotor.setIdleMode(IdleMode.kCoast);
    leftMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void positionFunction(double position){
    getController().reset(getArmAngle());
    m_controller.setGoal(position);
  }

  @Override
  public void updateTunableNumbers(){
    if (armKs.hasChanged(0)
      || armKv.hasChanged(0)
      || armKp.hasChanged(0)
      || armKi.hasChanged(0)
      || armKd.hasChanged(0)
      || armKa.hasChanged(0)) {
    
        m_controller = new ProfiledPIDController(armKp.get(), armKi.get(), armKd.get(), m_constraints);
  }
}
}

 