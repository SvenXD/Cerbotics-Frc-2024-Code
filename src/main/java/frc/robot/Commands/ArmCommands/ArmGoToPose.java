package frc.robot.Commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class ArmGoToPose extends Command {

  ArmSubsystem arm;

  public ArmGoToPose(ArmSubsystem arm) {
    this.arm = arm;

    addRequirements(arm);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    arm.test();
  }


  @Override
  public void end(boolean interrupted) {

  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
