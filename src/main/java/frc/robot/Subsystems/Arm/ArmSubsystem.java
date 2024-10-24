package frc.robot.Subsystems.Arm;

import static frc.robot.Constants.Arm.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Util.Interpolation.InterpolatingDouble;
import frc.Util.Interpolation.InterpolatingTreeMap;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

  /*Io and inputs */
  private final ArmIO io;
  private final ArmIoInputsAutoLogged inputs = new ArmIoInputsAutoLogged();

  /* Visualizer */
  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;
  private final ArmVisualizer goalVisualizer;

  /*Variables */
  private SendableChooser<String> armModeChooser = new SendableChooser<>();
  private String currentModeSelection;
  private final String[] modeNames = {"BRAKE", "COAST"};
  private Boolean enable = false;

  public static enum ArmStates {
    INTAKING,
    FEEDING,
    STANDING,
    SHOOTING,
    IDLE
  }

  private ArmStates systemStates = ArmStates.IDLE;

  static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToArmAngle =
      new InterpolatingTreeMap<>();

  static { // Added offset of 0 degrees
    kDistanceToArmAngle.put(new InterpolatingDouble(1.66), new InterpolatingDouble(160.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(2.05), new InterpolatingDouble(153.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(2.66), new InterpolatingDouble(143.5));
    kDistanceToArmAngle.put(new InterpolatingDouble(3.50), new InterpolatingDouble(138.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(4.15), new InterpolatingDouble(135.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(4.35), new InterpolatingDouble(134.0));
  }

  public ArmSubsystem(ArmIO io) {
    this.io = io;
    io.updateTunableNumbers();
    io.superSimPeriodic();
    Logger.processInputs("Arm", inputs);

    measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new ArmVisualizer("Goal", Color.kBlue);

    armModeChooser.setDefaultOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Coast Mode", modeNames[1]);

    SmartDashboard.putData("Arm Mode", armModeChooser);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.superSimPeriodic();

    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/Current State", systemStates.toString());

    if (DriverStation.isDisabled()) {
      currentModeSelection = armModeChooser.getSelected();
      switch (currentModeSelection) {
        case "BRAKE":
          io.setBrakeMode(true);
          break;

        case "COAST":
          io.setBrakeMode(false);
          break;
      }
    } else {
      io.setBrakeMode(false);
    }

    goalVisualizer.update(inputs.currentAngle);
    Logger.recordOutput("Arm/GoalAngle", inputs.setPoint);

    measuredVisualizer.update(getAngleRadiants());
    setpointVisualizer.update(inputs.currentAngle);
    Logger.recordOutput("Arm/SetpointAngle", inputs.setPoint);
    Logger.recordOutput("Arm/SetpointVelocity", inputs.pivotVel);
  }

  public Command goToPosition(Rotation2d position) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              io.setDesiredAngle(position);
            },
            this);
    return ejecutable;
  }

  public Command setOpenLoop(double position) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              io.setOpenLoop(position);
            },
            this);
    return ejecutable;
  }

  public double getAngleForDistance(double distance) {
    return kDistanceToArmAngle.getInterpolated(
            new InterpolatingDouble(Math.max(Math.min(distance, 4.35), 1.66)))
        .value;
  }

  public double getAngleRadiants() {
    return Units.degreesToRadians(inputs.currentAngle);
  }

  public ArmStates getState() {
    return systemStates;
  }

  public ArmStates changeState(ArmStates state) {
    systemStates = state;
    return systemStates;
  }

  public double getArmAngle() {
    return inputs.currentAngle;
  }
}
