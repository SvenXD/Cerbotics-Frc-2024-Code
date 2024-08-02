// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Util.LocalADStarAK;
import frc.Util.NoteVisualizer;
import frc.Util.Logging.LoggedDashboardChooser;
import frc.robot.Commands.AutoCommands.AutoCommand;
import frc.robot.Commands.AutoCommands.Paths.ChangeTest;
import frc.robot.Commands.AutoCommands.Paths.ComplementPath;
import frc.robot.Commands.AutoCommands.Paths.FiveNoteAutoPath;
import frc.robot.Commands.AutoCommands.Paths.NoneAuto;
import frc.robot.Commands.AutoCommands.Paths.TestAuto;
import frc.robot.Commands.SwerveCommands.DriveCommands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeSimIO;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Subsystems.Swerve.GyroIO;
import frc.robot.Subsystems.Swerve.GyroIOPigeon2;
import frc.robot.Subsystems.Swerve.ModuleIO;
import frc.robot.Subsystems.Swerve.ModuleIOSim;
import frc.robot.Subsystems.Swerve.ModuleIOTalonFX;

public class RobotContainer {

  //nopublic static SimDefenseBot defenseBot = new SimDefenseBot(2);

  private final static CommandXboxController chassisDriver = new CommandXboxController(0);
  private final static CommandXboxController subsystemsDriver = new CommandXboxController(1);

  private static LoggedDashboardChooser<AutoCommand> autoChooser;

  public static Field2d autoPreviewField = new Field2d();

  public static Drive drive;

    public static IntakeIO intakeIO = new IntakeSimIO();
  public static IntakeSubsystem m_intake = new IntakeSubsystem(intakeIO);

  public RobotContainer() {
  /** Options for the current mode of the robot */
     switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
      drive = new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));   

        break;
        //--------------------------------------------
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        break;
        //--------------------------------------------
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
        //--------------------------------------------
    }

    /** Visualisation of the current auto selected **/
    autoChooser = new LoggedDashboardChooser<>("Auto Mode");

    autoChooser.onChange(
        auto -> {
          autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());});

    /**Auto options */      
    autoChooser.addDefaultOption("None", new NoneAuto());
    autoChooser.addOption("Complement auto", new ComplementPath());
    autoChooser.addOption("Six Note Auto", new FiveNoteAutoPath());
    autoChooser.addOption("Test Of Change", new ChangeTest(drive));
    autoChooser.addOption("Test", new TestAuto());

    PathPlannerLogging.setLogActivePathCallback(
      (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
    PathPlannerLogging.setLogTargetPoseCallback(
      pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));
      
    SmartDashboard.putData("Auto Preview", autoPreviewField);

    SmartDashboard.putString("Current Robot mode", Constants.currentMode.toString());

    Pathfinding.setPathfinder(new LocalADStarAK());

    // Set up note visualizer
    NoteVisualizer.setRobotPoseSupplier(drive::getPose);

    configureBindings();

  }

  private void configureBindings() {

  /* Control 1 commands */
    //Chassis commands
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -chassisDriver.getLeftY(),
            () -> -chassisDriver.getLeftX(),
            () -> -chassisDriver.getRightX()));

      //Lock modules
    chassisDriver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    
      //Set field centric
    chassisDriver.a().onTrue(drive.runOnce(() -> drive.zeroHeading()));

    chassisDriver.leftBumper().whileTrue(m_intake.goToPosition(0)).whileFalse(m_intake.goToPosition(90));

  }

   private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          chassisDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          chassisDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
     }

   public static Command pathfindAndAlignAmp() {
    return Commands.either(
            drive.goToPose(
              FieldConstants.redAmpPose)
            .until(
              () ->
              Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
            drive.goToPose(FieldConstants.blueAmpPose)
            .until(
              () ->
            Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
            Robot::isRedAlliance);
          }

   public static Command pathfindAndAlignSource() {
    return Commands.either(
            drive.goToPose(
              FieldConstants.redPickupPose)
            .until(
              () ->
              Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
            drive.goToPose(FieldConstants.bluePickupPose)
            .until(
              () ->
               Math.abs(chassisDriver.getRawAxis(1)) > 0.1),
            Robot::isRedAlliance);
          }    
          
      
          
  public Command getAutonomousCommand() {
    return autoChooser.get();

  }
  
  public static Drive getSwerveSubsystem(){
    return drive;
  }
}
