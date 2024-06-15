package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem.ShooterState;

public class AmpShoot extends Command {

  ShooterSubsystem shooter;

  public AmpShoot(ShooterSubsystem shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    shooter.requestAMP();
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
