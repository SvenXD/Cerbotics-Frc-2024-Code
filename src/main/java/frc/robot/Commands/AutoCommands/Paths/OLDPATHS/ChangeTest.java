package frc.robot.Commands.AutoCommands.Paths.OLDPATHS;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Subsystems.Swerve.Drive;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class ChangeTest extends AutoCommand {

  Drive m_drive;
  private final PathPlannerPath startToFirst = PathPlannerPath.fromPathFile("Test1");
  private final PathPlannerPath startToSecond = PathPlannerPath.fromPathFile("Test3");
  private final PathPlannerPath startToSecondAlt = PathPlannerPath.fromPathFile("Test2");
  private final PathPlannerPath startToSecondAltAlt = PathPlannerPath.fromPathFile("Test4");

  private final PathPlannerPath idk = PathPlannerPath.fromPathFile("Test5");

  public ChangeTest(Drive m_drive) {
    this.m_drive = m_drive;
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(startToFirst.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getStartingPose() {
    return startToFirst
        .getTrajectory(new ChassisSpeeds(), new Rotation2d())
        .getInitialTargetHolonomicPose();
  }
}
