package frc.robot.Subsystems.Pivot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class PivotVisualizer {
  private final Mechanism2d mechanism;
  private final MechanismLigament2d pivot;
  private final String key;

  public PivotVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    MechanismRoot2d root = mechanism.getRoot("pivot", 1.68, 0.1);
    pivot = new MechanismLigament2d("pivot", 0.6, 20.0, 6, new Color8Bit(color));
    root.append(pivot);
  }

  /** Update arm visualizer with current arm angle */
  public void update(double angleDegr) {
    // Log Mechanism2d
    pivot.setAngle(Rotation2d.fromDegrees(angleDegr));
    Logger.recordOutput("Pivot/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d pivot = new Pose3d(0.1605, 0.2, 0.262, new Rotation3d(0.0, -angleDegr, 0.0));
    Logger.recordOutput("Pivot/Mechanism3d/" + key, pivot);
  }
}
