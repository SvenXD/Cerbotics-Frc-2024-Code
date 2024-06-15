package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class OverStageShoot extends Command {

  ShooterSubsystem shooter;

  public OverStageShoot(ShooterSubsystem shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    shooter.requestHigh_Pass();
  }


  @Override
  public void end(boolean interrupted) {
    shooter.requestIdle();
    shooter.unsetAllRequests();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
