package frc.robot.Subsystems.ArmExtention;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;


public class ArmExtentionIOSim implements ArmExtentionIO{
  
  private final ElevatorSim sim = new ElevatorSim(
    0.11515,
    0.0017514,
    DCMotor.getFalcon500Foc(1),
    0.0,
    0.42,
    false,
    0.0); 

  public double appliedVolts = 0.0;

    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", sim.getPositionMeters(), 90));
    public ArmExtentionIOSim(){
        sim.setState(0.0, 0.0);
    }
    
  @Override
  public void updateInputs(ArmExtentionIOInputs inputs) {
    sim.update(0.02);

    inputs.currentPosition = sim.getPositionMeters();
    inputs.appliedVolts = sim.getCurrentDrawAmps();
    m_elevatorMech2d.setLength(sim.getPositionMeters());
    Logger.recordOutput("ArmExtention/Mech2d", m_mech2d);

    sim.setInputVoltage(0.0);

  }

  @Override
  public void setVoltage(ProfiledPIDController m_controller, ElevatorFeedforward m_feedforward) {
    runVolts( m_controller.calculate(sim.getPositionMeters()) + m_feedforward.calculate(m_controller.getSetpoint().velocity));
  }
  @Override
  public void runVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(volts);
  }

  @Override
  public void update(double angleRads) {
    m_elevatorMech2d.setAngle(Rotation2d.fromRadians(angleRads));
    Logger.recordOutput("Arm/ArmExtention/Mechanism2d/", m_mech2d);

    Pose3d pivot =
      new Pose3d(-0.2, 0.0,0.31
       , new Rotation3d(0.0, -angleRads + 0.226893, 0.0));
       
    Logger.recordOutput("Arm/ArmExtention/Mechanism3d/", getArmDistalPose(pivot));
    }


   public  Pose3d getArmDistalPose(Pose3d armProximalPose) {
    double armExtensionLength = sim.getPositionMeters();
    return armProximalPose.transformBy(
        new Transform3d(new Translation3d(armExtensionLength, 0.0, 0.0), new Rotation3d()));
  }

}