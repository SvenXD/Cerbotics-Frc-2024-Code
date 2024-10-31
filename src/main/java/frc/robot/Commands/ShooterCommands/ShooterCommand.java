package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class ShooterCommand extends Command {

  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;

  public ShooterCommand(ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
    this.m_shooter = m_shooter;
    this.m_intake = m_intake;

    addRequirements(m_shooter, m_intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.setRpmsVoid(90, 90);

    if (m_shooter.getRPM() > 4000) {
      m_intake.setUpperIntakeVoltageVoid(-1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_intake.setUpperIntakeVoltageVoid(0);
  }

  @Override
  public boolean isFinished() {
    return !m_intake.isNoteInside();
  }
}
