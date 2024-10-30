package frc.robot.Commands.AutoCommands.Paths.OLDPATHS;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.AutoCommands.AutoCommand;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class TestAuto extends AutoCommand {

  private final PathPlannerPath first;
  private final PathPlannerPath second;
  private final PathPlannerPath third;
  private final PathPlannerPath fourth;
  private final PathPlannerPath fifth;
  private final PathPlannerPath sixth;

  public TestAuto() {
    first = PathPlannerPath.fromPathFile("Intake1");
    second = PathPlannerPath.fromPathFile("Intake2");
    third = PathPlannerPath.fromPathFile("Intake3");
    fourth = PathPlannerPath.fromPathFile("Intake4");
    fifth = PathPlannerPath.fromPathFile("Intake5");
    sixth = PathPlannerPath.fromPathFile("Intake6");

    addCommands(
        Commands.deadline(
            Commands.sequence(
                new PathPlannerAuto("Starting pose 1"), new PathPlannerAuto("IntakeAssistAuto"))));
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
            first.getPathPoses(),
            second.getPathPoses(),
            third.getPathPoses(),
            fourth.getPathPoses(),
            fifth.getPathPoses(),
            sixth.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getStartingPose() {
    return first
        .getTrajectory(new ChassisSpeeds(), new Rotation2d())
        .getInitialTargetHolonomicPose();
  }
}
