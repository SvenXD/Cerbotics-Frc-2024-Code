package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import static frc.robot.Constants.Shooter.*;

public class UnderStageShoot extends Command {

  ShooterSubsystem shooter;

  public UnderStageShoot(ShooterSubsystem shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    shooter.velocity(UPPER_SHOOTER_FEEDER_UNDER_RPM, LOWER_SHOOTER_FEEDER_UNDER_RPM);
  }


  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
