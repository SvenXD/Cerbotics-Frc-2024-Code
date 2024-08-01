// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean e = false;
  private final TrapezoidProfile.Constraints m_constraints = 
    new TrapezoidProfile.Constraints(1000,1000);

  private ProfiledPIDController m_controller = new ProfiledPIDController(0.32, 0.42, 0.0055, m_constraints);

  private final ArmFeedforward m_feedforward = new ArmFeedforward(0.013804, 0.00028699, 0.93532,0.00052411);

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
  io.updateInputs(inputs);
  Logger.processInputs("Intake", inputs); 
  if(e){
   io.setIntakePosition(m_controller.calculate(inputs.positionDegrees), 
   m_feedforward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity));
  }
 }

 public Command goToPosition(double position){
  Command ejecutable = Commands.runOnce(
    () -> {
      m_controller.reset(inputs.positionDegrees);
      m_controller.setGoal(position);
      e = true;
    },
    this);
return ejecutable;
 }
}
