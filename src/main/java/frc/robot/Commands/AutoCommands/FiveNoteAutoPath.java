package frc.robot.Commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class FiveNoteAutoPath extends AutoCommand {

  private final PathPlannerPath startToFirst;
  private final PathPlannerPath secondToFirst;

  public FiveNoteAutoPath() {
    startToFirst = PathPlannerPath.fromPathFile("5NoteAuto1");
    secondToFirst = PathPlannerPath.fromPathFile("5NoteAuto2");

    addCommands(
        Commands.deadline(
            Commands.sequence(
                new PathPlannerAuto("5NoteAuto")
    )));

  }


  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
            startToFirst.getPathPoses(),
            secondToFirst.getPathPoses()
    )
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