package frc.robot.Commands.AutoCommands.Paths;

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

  private final PathPlannerPath startToFirst;


  public TestAuto() {
    startToFirst = PathPlannerPath.fromPathFile("5NoteAuto1");


    addCommands(
        Commands.deadline(
            Commands.sequence(
                new PathPlannerAuto("New Auto")
    )));

  }


  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
            startToFirst.getPathPoses()

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