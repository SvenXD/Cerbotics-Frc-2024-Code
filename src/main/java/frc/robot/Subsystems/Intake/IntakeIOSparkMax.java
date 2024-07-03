package frc.robot.Subsystems.Intake;

import static frc.robot.Constants.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO{
    /* Hardware */
    private final CANSparkMax intakeMotor;
    public DigitalInput intakeSensor;
    public RelativeEncoder intakeEncoder;

    public IntakeIOSparkMax(){

        intakeMotor = new CANSparkMax(INTAKE_ID, MotorType.kBrushless);
        intakeSensor = new DigitalInput(INTAKE_SENSOR_ID);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setCANTimeout(250);

        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(40);

        intakeMotor.setInverted(INTAKE_INVERSION);
        intakeMotor.setCANTimeout(0);

        intakeEncoder = intakeMotor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.positionRad = intakeEncoder.getPosition();
        inputs.appliedVolts = intakeMotor.getBusVoltage();
        inputs.velocityRadPerSec = intakeEncoder.getVelocity();
        inputs.tempCelcius = intakeMotor.getMotorTemperature();
        inputs.sensor = !intakeSensor.get();
    }

    @Override
    public void setVoltage(double volts){
        intakeMotor.set(volts);
    }
}
