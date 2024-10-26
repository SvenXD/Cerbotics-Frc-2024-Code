package frc.robot.Subsystems.IntakexAbajo;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Arm.ArmSubsystem.ArmStates;
import frc.robot.Subsystems.Shooter.ShooterIOInputsAutoLogged;

public class IntakexAbSubsystem extends SubsystemBase{

    private final IntakeAbIO io;
    private final IntakeAbIOInputsAutoLogged inputs = new IntakeAbIOInputsAutoLogged();

    public IntakexAbSubsystem(IntakeAbIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
      
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("IntakeAb", inputs);
    Logger.recordOutput("IntakeAb/RPMS", inputs.appliedVolts);
    }

    public void velocity(double intakeAb) {
        io.setVelocity(intakeAb);
      }
    
      public void stopMotors() {
        io.stop();
      }
      
      public Command setVoltage(double voltage) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
             velocity(voltage);
            },
            this);
    return ejecutable;
  }
}
