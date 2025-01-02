// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands.AutomatedStuff;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class AmpRoutine extends Command {

  private ArmSubsystem m_arm;
  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;
  private Timer timer = new Timer();

  public AmpRoutine(ArmSubsystem m_arm, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
    this.m_arm = m_arm;
    this.m_shooter = m_shooter;
    this.m_intake = m_intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_arm.updateArmSetpoint(95);
    m_shooter.setRpmsVoid(11, 11);

    if (m_arm.getArmAngle() < 96) {
      timer.start();
      m_intake.setUpperIntakeVoltageVoid(-1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.goToPositionVoid(173);
    m_shooter.stop();
    m_intake.setUpperIntakeVoltageVoid(0);
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return !m_intake.isNoteInside() && timer.get() > 1;
  }
}
