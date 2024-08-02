// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmExtension;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;


  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          ArmExtension.kP,
          ArmExtension.kI,
          ArmExtension.kD,
          new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
        ArmExtension.kS,
        0,
        ArmExtension.kV,
          ArmExtension.kA);


  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ArmExtention", inputs);
    io.setVoltage(controller, m_feedforward);
    io.update(89.9);   // + --->    - <------
  }

      public Command goToPosition(double pose){
    Command ejecutable = Commands.runOnce(
                () -> {
                getController().reset(inputs.currentPosition);
                controller.setGoal(pose);
                },
                this);
    return ejecutable;
  }

  public ProfiledPIDController getController(){
    return controller;
  }
}