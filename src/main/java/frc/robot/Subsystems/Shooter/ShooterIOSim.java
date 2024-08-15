package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private FlywheelSim sim = new FlywheelSim(DCMotor.getKrakenX60(2), 1, 0.004);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.upperShooterPosRad = sim.getAngularVelocityRPM();
    inputs.upperShooterVelocityRpm = sim.getAngularVelocityRadPerSec();
    inputs.upperShooterAppliedVolts = appliedVolts;
    inputs.shooterCurrentAmps = new double[] {sim.getCurrentDrawAmps()};

    inputs.lowerShooterAppliedVolts = sim.getAngularVelocityRPM();
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityUp, double velocityDown) {
    closedLoop = true;
    pid.setSetpoint(velocityUp);
    this.ffVolts = velocityDown;
  }
}
