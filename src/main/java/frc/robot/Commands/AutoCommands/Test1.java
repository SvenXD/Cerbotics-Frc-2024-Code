package frc.robot.Commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Test1 extends AutoCommand {

  private final PathPlannerPath startToFirst;
  private final PathPlannerPath firstToFarShoot;

  public Test1() {
    startToFirst = PathPlannerPath.fromPathFile("Path1");
    firstToFarShoot = PathPlannerPath.fromPathFile("Path2");

    addCommands(
        Commands.deadline(
            Commands.sequence(
                AutoBuilder.followPath(startToFirst),
                AutoBuilder.followPath(startToFirst))));
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
            startToFirst.getPathPoses(),
            firstToFarShoot.getPathPoses()
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