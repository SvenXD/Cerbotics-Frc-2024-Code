package frc.robot.Subsystems.Arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ArmIOSparkMax implements ArmIO{

    private final CANSparkMax leftMotor = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(10, MotorType.kBrushless);

    private final CANcoder m_encoder = new CANcoder(17, "Swerve_Canivore");


     CANcoderConfiguration encoderConfig = new CANcoderConfiguration();


    public ArmIOSparkMax(){

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

  
}
