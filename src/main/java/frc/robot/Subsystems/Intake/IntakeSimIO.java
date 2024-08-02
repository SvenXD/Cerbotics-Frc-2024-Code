package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeSimIO implements IntakeIO{
    private static final double autoStartAngle = 90.0;


    private final SingleJointedArmSim sim =
    new SingleJointedArmSim(
        DCMotor.getNEO(2), 
        18, 
        1.1, 
        0.6, 
        Units.degreesToRadians(0), 
        Units.degreesToRadians(93), 
        false, 
        Units.degreesToRadians(autoStartAngle));

    private final PIDController controller;
    private double appliedVoltage = 0.0;
    private double positionOffset = 0.0;

    private boolean controllerNeedsReset = false;
    private boolean closedLoop = true;
    private boolean wasNotAuto = false;

    public IntakeSimIO(){
      controller = new PIDController(0.5, 0.0, 0.1);
        sim.setState(0.0, 0.0);
        setPosition(0.0);

    }
    
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    visualizer();
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

    inputs.positionDegrees = Units.radiansToDegrees(sim.getAngleRads()) + positionOffset;
    inputs.intakeCurrentAmps = appliedVoltage;

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void setIntakePosition(double output, double feedforward) {
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }
    runVolts(controller.calculate(sim.getAngleRads(), output - positionOffset) + feedforward);
  }

  public void runVolts(double volts) {
    closedLoop = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(volts);
  }

    private void setPosition(double position) {
        positionOffset = position - sim.getAngleRads();
      }    

   public void visualizer(){
    Pose3d pivot = new Pose3d(-0.32,-0.3,0.12, new Rotation3d(0, sim.getAngleRads(),0));
    Logger.recordOutput("Intake/Mechanism3d/", pivot);

   }   
}
