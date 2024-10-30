package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class IntakeWithSensor extends Command {

  IntakeSubsystem intake;

  public IntakeWithSensor(IntakeSubsystem intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setallVoid(-0.7, -0.8);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setallVoid(0, 0);
  }

  @Override
  public boolean isFinished() {
    return intake.isNoteInside();
  }
}
