package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class SpeakerShoot extends Command {

  ShooterSubsystem shooter;

  public SpeakerShoot(ShooterSubsystem shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    shooter.test(2400,2400);
  }


  @Override
  public void end(boolean interrupted) {
    shooter.test2();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
