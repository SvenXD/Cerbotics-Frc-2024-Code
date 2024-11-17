// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {

  private final TalonFX tractionMotor = new TalonFX(10, "Swerve_Canivore");
  private static TalonFXConfiguration intakeConfig;
  private final VoltageOut heyg;

  public TestSubsystem() {
    intakeConfig = new TalonFXConfiguration();
    heyg = new VoltageOut(0);

    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 80;
    intakeConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    intakeConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    /* Apply Configurations*/
    tractionMotor.getConfigurator().apply(intakeConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FOC TEST", tractionMotor.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putString("oisdfhuiweonm", heyg.toString());
  }

  public Command setVoltage(double armVolt) {
    return run(() -> tractionMotor.setControl(heyg.withOutput(armVolt).withEnableFOC(false)));
  }
}
