// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ArmJoint;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Arm.*;

public class ArmJointSubsystem extends SubsystemBase {
  private final ArmJointIO io;
  private final ArmJointIOInputsAutoLogged inputs;
  private Rotation2d targetRotation = Rotation2d.fromDegrees(13);
  private Boolean enable = false;

    /* Visualizer */
  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;
  private final ArmVisualizer goalVisualizer;

    private final TrapezoidProfile.Constraints m_constraints;
    private ProfiledPIDController m_controller;

    private final ArmFeedforward m_feedforward;
  public ArmJointSubsystem(ArmJointIO io) {
    this.io = io;
    this.inputs = new ArmJointIOInputsAutoLogged();

    m_feedforward = new ArmFeedforward(kS, kG,kV,kA);
    m_constraints = new TrapezoidProfile.Constraints(0.0,0.0);
    m_controller = new ProfiledPIDController(kP,kI,kD,m_constraints,kPeriod);

    measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new ArmVisualizer("Goal", Color.kBlue);

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ArmJoint", inputs);

    Logger.recordOutput("ArmJoint/TargetAngle", targetRotation);

    if(enable){
      io.setVoltage(m_controller.calculate(inputs.currentAngle), 
      m_feedforward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity));
     }

     goalVisualizer.update(getController().getGoal().position);
     Logger.recordOutput("ArmJoint/GoalAngle", getController().getGoal().position);
 
     measuredVisualizer.update(getAngleRadiants());
     setpointVisualizer.update(m_controller.getSetpoint().position);
     Logger.recordOutput("ArmJoint/SetpointAngle", m_controller.getSetpoint().position);
     Logger.recordOutput("ArmJoint/SetpointVelocity", m_controller.getSetpoint().velocity);

    }

    public Command goToPosition(double pose){
    Command ejecutable = Commands.runOnce(
                () -> {
                getController().reset(inputs.currentAngle);
                m_controller.setGoal(pose);
                enable = true;
                },
                this);
    return ejecutable;
  }

  public ProfiledPIDController getController(){
    return m_controller;
  }

  public double getAngleRadiants(){
    return Units.degreesToRadians(inputs.currentAngle);
   }
}
