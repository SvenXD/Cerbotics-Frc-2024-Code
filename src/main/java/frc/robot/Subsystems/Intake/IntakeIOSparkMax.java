package frc.robot.Subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO{

    private boolean invert = false;
    private final CANSparkMax intakeMotor;

    public DigitalInput intakeSensor = new DigitalInput(0);

    public RelativeEncoder intakeEncoder;


    public IntakeIOSparkMax(){

        intakeMotor = new CANSparkMax(12, MotorType.kBrushless);

        intakeMotor.setIdleMode(IdleMode.kBrake);

        intakeMotor.setInverted(invert);

        intakeMotor.setSmartCurrentLimit(80);

        intakeEncoder = intakeMotor.getEncoder();

    }
    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.positionRad = intakeEncoder.getPosition();
        inputs.appliedVolts = intakeMotor.getBusVoltage();
        inputs.velocityRadPerSec = intakeEncoder.getVelocity();
        inputs.tempCelcius = intakeMotor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts){
        intakeMotor.set(volts);
    }

    @Override
    public void setBrakeMode(boolean enable){
        intakeMotor.setIdleMode(enable ?IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setCurrentLimit(int currentLimit){
        intakeMotor.setSmartCurrentLimit(currentLimit);
    }
}
