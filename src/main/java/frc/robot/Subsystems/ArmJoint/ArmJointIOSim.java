package frc.robot.Subsystems.ArmJoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.system.LinearSystem;  Interesting, check later
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmJointIOSim implements ArmJointIO{
    private static final double autoStartAngle = 110.0;


    private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getFalcon500Foc(2),  SingleJointedArmSim.estimateMOI(Units.inchesToMeters(20.0), 12), 3125.0 / 27.0),
        DCMotor.getFalcon500Foc(2), 
        3125.0 / 27.0, 
        0.508, 
        Units.degreesToRadians(13), 
        Units.degreesToRadians(180), 
        false, 
        Units.degreesToRadians(13));

    private final PIDController controller;
    private double appliedVoltage = 0.0;
    private double positionOffset = 0.0;

    private boolean controllerNeedsReset = false;
    private boolean closedLoop = true;
    private boolean wasNotAuto = false;

    public ArmJointIOSim(){
      controller = new PIDController(0.32, 0.42, 0.0055);
        sim.setState(0.0, 0.0);
        setPosition(0.0);

    }
    
  @Override
  public void updateInputs(ArmJointIOInputs inputs) {
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