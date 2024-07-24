package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO{
    private static final double autoStartAngle = 110.0;


    private final SingleJointedArmSim sim =
    new SingleJointedArmSim(
        DCMotor.getNEO(2), 
        120, 
        1.1, 
        0.6, 
        Units.degreesToRadians(90), 
        Units.degreesToRadians(185), 
        false, 
        Units.degreesToRadians(autoStartAngle));

    private final PIDController controller;
    private double appliedVoltage = 0.0;
    private double positionOffset = 0.0;

    private boolean controllerNeedsReset = false;
    private boolean closedLoop = true;
    private boolean wasNotAuto = false;

    public ArmIOSim(){
      controller = new PIDController(0.32, 0.42, 0.0055);
        sim.setState(0.0, 0.0);
        setPosition(0.0);

    }
    
  @Override
  public void updateInputs(ArmIoInputs inputs) {
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // Reset at start of auto
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(autoStartAngle, 0.0);
      wasNotAuto = false;
    }
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    sim.update(0.02);

    inputs.currentAngle = Units.radiansToDegrees(sim.getAngleRads()) + positionOffset;
    inputs.leftAppliedVolts = appliedVoltage;
    inputs.rightAppliedVolts = appliedVoltage;
    inputs.leftTempCelcius = 0.0;
    inputs.rightTempCelcius = 0.0;

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void setVoltage(double output, double feedforward) {
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }
    runVolts(controller.calculate(sim.getAngleRads(), output + positionOffset) + feedforward);
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(volts);
  }


  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }

    private void setPosition(double position) {
        positionOffset = position - sim.getAngleRads();
      }    
}
