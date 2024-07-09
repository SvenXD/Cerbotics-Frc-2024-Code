// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Util.AutoSelector;
import frc.Util.AutoSelector.AutoQuestion;
import frc.Util.AutoSelector.AutoQuestionResponse;
import frc.robot.Commands.AutoCommands.AutoCommands;
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Commands.AutoCommands.NoneAuto;
import frc.robot.Commands.AutoCommands.Test1;
import frc.robot.Commands.AutoCommands.Test2;
import frc.Util.LoggedDashboardChooser;


public class RobotContainer {
  private static LoggedDashboardChooser<AutoCommand> autoChooser = new LoggedDashboardChooser<>("Auto Mode");

  public static Field2d autoPreviewField = new Field2d();
  private final AutoSelector autoSelector = new AutoSelector("Auto");

  public RobotContainer() {
 
    /** Visualisation of the current auto selected **/
    autoChooser.onChange(
        auto -> {
          autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());});

    /**Auto options */      
    autoChooser.addDefaultOption("None", new NoneAuto());
    autoChooser.addOption("Test1", new Test1());
    autoChooser.addOption("Test2", new Test2());
    
    SmartDashboard.putData("Auto Preview", autoPreviewField);
    AutoCommands autoCommands =
    new AutoCommands(autoSelector::getResponses);


    autoSelector.addRoutine(
        "Field:givgvgnlkñmlkms<e<io",
        List.of(
            new AutoQuestion(
                "Which level?",
                List.of(
                    AutoQuestionResponse.HIGH,
                    AutoQuestionResponse.MID,
                    AutoQuestionResponse.HYBRID)),
            new AutoQuestion(
                "End behavior?",
                List.of(
                    AutoQuestionResponse.RETURN,
                    AutoQuestionResponse.BALANCE,
                    AutoQuestionResponse.BALANCE_THROW))),
        autoCommands.fieldScoreTwoGrabMaybeBalance());


    configureBindings();
  }

  private void configureBindings() {



  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
