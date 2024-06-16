package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class IntakeCommand extends Command {

  IntakeSubsystem intake;

  public IntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;

    addRequirements(intake);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    intake.requestIntake(1.0);
  }


  @Override
  public void end(boolean interrupted) {
    intake.requestIdle();
    intake.unsetAllRequests();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
