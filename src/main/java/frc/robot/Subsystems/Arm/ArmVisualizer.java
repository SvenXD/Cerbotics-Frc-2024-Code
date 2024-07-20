package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmVisualizer {
  private final Mechanism2d mechanism;
  private final MechanismLigament2d arm;
  private final String key;

  public ArmVisualizer(String key, Color color){
    this.key = key;
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
     MechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);
    arm = new MechanismLigament2d("arm", 0.6, 20.0, 6, new Color8Bit(color));
    root.append(arm);
  }

    /** Update arm visualizer with current arm angle */
  public void update(double angleRads) {
    // Log Mechanism2d
    arm.setAngle(Rotation2d.fromRadians(angleRads));
    Logger.recordOutput("Arm/Mechanism2d/" + key, mechanism);
  }
}
