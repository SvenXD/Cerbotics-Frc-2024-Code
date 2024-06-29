// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.Subsystems.Arm;

import javax.swing.text.Position;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Armtest extends SubsystemBase {
   Creates a new Armtest. 

    private final CANSparkMax leftMotor = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(10, MotorType.kBrushless);
 
    private final CANcoder m_encoder = new CANcoder(17, "Swerve_Canivore");

      private boolean m_enabled = false;
 
    private final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(500, 500);

    private final ProfiledPIDController m_controller =
    new ProfiledPIDController(0.32, 0.42, 0.0055, m_constraints, 0.02);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(0.013804, 0.93532, 0.00028699, 0.00052411);
        
    private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        private SendableChooser<String> armModeChooser = new SendableChooser<>();
    private String currentModeSelection;
    private final String[] modeNames = {"BRAKE", "COAST"};

  public Armtest() {


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
        
    armModeChooser.setDefaultOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Coast Mode", modeNames[1]);

    SmartDashboard.putData("Arm Mode", armModeChooser);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("test angle", getArmAngle());
    SmartDashboard.putNumber("Test setpoint", m_controller.getGoal().position);

    if (m_enabled) {
      setVoltage(m_controller.calculate(getArmAngle()),m_controller.getSetpoint());
    }

      if(DriverStation.isDisabled()){
        currentModeSelection = armModeChooser.getSelected();
        switch (currentModeSelection) {
          case "BRAKE":
            armSetBrake();
          break;

          case "COAST":
            armSetCoast();
          break;
        }
      } else {
        armSetBrake();
      }
    
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
    leftMotor.setVoltage(output+ feedfoward);
  }

  public void armSetCoast(){
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);  
   }
 
   public void armSetBrake(){
     leftMotor.setIdleMode(IdleMode.kBrake);
     rightMotor.setIdleMode(IdleMode.kBrake);  
   }

  public Command goToPosition(double position){
    Command ejecutable = Commands.runOnce(
                () -> {
                getController().reset(getArmAngle());
                m_controller.setGoal(position);
                enable();                                          //LEAVE ENABEL
                },
                this);
    return ejecutable;
  }

  public void enable() {
    m_enabled = true;
    m_controller.reset(getArmAngle());
  }

}
*/