// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final PivotVisualizer measuredVizualiser = new PivotVisualizer("Measured", Color.kAqua);

  public PivotSubsystem(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("Pivot", inputs);

    if (Robot.isSimulation()) {
      io.superSimPeriodic();
    }

    measuredVizualiser.update(inputs.pivotPositionDeg);
  }

  public void setVoltage(double volt) {
    io.setOpenLoop(volt);
  }

  public Command setAngle(Rotation2d angle) {
    return this.run(() -> io.setDesiredAngle(Degrees.of(angle.times(1).getDegrees())));
  }
}
