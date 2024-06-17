package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class Outake extends Command {

  IntakeSubsystem intake;
  ShooterSubsystem shooter;

  public Outake(IntakeSubsystem intake, ShooterSubsystem shooter) {
    this.intake = intake;
    this.shooter = shooter;

    addRequirements(intake);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    intake.requestIntake(-0.2);
    shooter.requestCustom(-900, -500);
  }


  @Override
  public void end(boolean interrupted) {
    intake.requestIdle();
    intake.unsetAllRequests();
    shooter.requestIdle();
    shooter.unsetAllRequests();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
