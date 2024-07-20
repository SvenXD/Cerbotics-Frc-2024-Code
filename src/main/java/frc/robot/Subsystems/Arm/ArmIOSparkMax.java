package frc.robot.Subsystems.Arm;

import com.revrobotics.CANSparkMax;

import static frc.robot.Constants.Arm.*;

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
  
  @Override
  public void updateInputs(ArmIoInputs inputs){
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput();
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput();
    inputs.leftTempCelcius = leftMotor.getMotorTemperature();
    inputs.rightTempCelcius = rightMotor.getMotorTemperature();

    inputs.currentAngle = getArmAngle();
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
    
  public double getArmAngle() {            
    return (m_encoder.getAbsolutePosition().getValueAsDouble() * 360)  + 50.6;
  }

  @Override
  public void setVoltage(double output, double feedfoward){
    rightMotor.setVoltage(output + feedfoward);
    leftMotor.setVoltage(output + feedfoward);
  }
}

 