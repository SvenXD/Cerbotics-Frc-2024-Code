package frc.robot.Subsystems.Intake;

import static frc.robot.Constants.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO{

    private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_ID, MotorType.kBrushless);
    public DigitalInput intakeSensor = new DigitalInput(INTAKE_SENSOR_ID);
    public RelativeEncoder intakeEncoder;

    public IntakeIOSparkMax(){
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(INTAKE_INVERSION);
        intakeMotor.setSmartCurrentLimit(80);
        intakeEncoder = intakeMotor.getEncoder();
    }

    public boolean noteSensor(){
        return !intakeSensor.get();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.positionRad = intakeEncoder.getPosition();
        inputs.appliedVolts = intakeMotor.getBusVoltage();
        inputs.velocityRadPerSec = intakeEncoder.getVelocity();
        inputs.tempCelcius = intakeMotor.getMotorTemperature();
        inputs.sensor = noteSensor();
    }

    @Override
    public void setVoltage(double volts){
        intakeMotor.set(volts);
    }

}
