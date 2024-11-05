package frc.robot.Commands.AutoCommands.Paths.RegionalPaths;

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

public class MoveTest extends AutoCommand {

  private final PathPlannerPath first = PathPlannerPath.fromPathFile("4NoteAuto1");
  private final PathPlannerPath second = PathPlannerPath.fromPathFile("4NoteAuto2");
  private final PathPlannerPath third = PathPlannerPath.fromPathFile("4NoteAuto3");
  private final PathPlannerPath fourth = PathPlannerPath.fromPathFile("4NoteAuto4");
  private final PathPlannerPath fifth = PathPlannerPath.fromPathFile("4NoteAuto5");
  private final PathPlannerPath sixth = PathPlannerPath.fromPathFile("4NoteAuto6");
  private final PathPlannerPath seventh = PathPlannerPath.fromPathFile("4NoteAuto7");

  public MoveTest() {

    addCommands(Commands.deadline(Commands.sequence(new PathPlannerAuto("4NoteAuto"))));
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
            first.getPathPoses(),
            second.getPathPoses(),
            third.getPathPoses(),
            fourth.getPathPoses(),
            fifth.getPathPoses(),
            sixth.getPathPoses(),
            seventh.getPathPoses())
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
