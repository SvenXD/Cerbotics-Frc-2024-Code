// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.AutoCommands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.Util.AutoSelector.AutoQuestionResponse;

public class AutoCommands {

  private final Supplier<List<AutoQuestionResponse>> responses;

  public AutoCommands(Supplier<List<AutoQuestionResponse>> responses) {
    this.responses = responses;

  }

  public Command fieldScoreTwoGrabMaybeBalance() {
    return select(
        Map.of(
            AutoQuestionResponse.HYBRID,
            test(),
            AutoQuestionResponse.MID,
            test(),
            AutoQuestionResponse.HIGH,
            test()),
        () -> responses.get().get(0));
  }

  public Command test(){
    return Commands.print("No autonomous command configured");
  }
}