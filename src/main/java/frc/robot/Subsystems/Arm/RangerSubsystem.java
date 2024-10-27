package frc.robot.Subsystems.Arm;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class RangerSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmInputsAutoLogged inputs;

  private final SysIdRoutine sysIdRoutine;

  private Rotation2d targetRotation = Rotation2d.fromDegrees(90);

  /*Variables */
  private SendableChooser<String> armModeChooser = new SendableChooser<>();
  private String currentModeSelection;
  private final String[] modeNames = {"BRAKE", "COAST"};

  public RangerSubsystem(ArmIO io) {
    this.io = io;
    this.inputs = new ArmInputsAutoLogged();

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> setManual(volts.in(Volts)), null, this));

    armModeChooser.setDefaultOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Coast Mode", modeNames[1]);

    SmartDashboard.putData("Arm Mode", armModeChooser);
  }

  @Override
  public void periodic() {
    long startTime = Logger.getRealTimestamp();

    io.updateInputs(inputs);
    Logger.processInputs("ArmJoint", inputs);

    Logger.recordOutput("ArmJoint/TargetAngle", targetRotation);

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("ArmJoint/PeriodicRuntimeMS", runtimeMS);

    if (DriverStation.isDisabled()) {
      currentModeSelection = armModeChooser.getSelected();
      switch (currentModeSelection) {
        case "BRAKE":
          io.enableBreak(true);
          break;

        case "COAST":
          io.enableBreak(false);
          break;
      }
    } else {
      io.enableBreak(false);
    }
  }

  /**
   * Set the target angle for the arm joint
   *
   * @param angle Target arm angle
   */
  public void setTargetAngle(Rotation2d angle) {
    targetRotation = angle;
    io.setTargetAngle(targetRotation);
  }

  public void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel) {
    targetRotation = angle;
    io.setTargetAngle(targetRotation, maxVel, maxAccel);
  }

  /**
   * Set the voltage output to the arm joint motors
   *
   * @param volts Voltage to output
   */
  public void setManual(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Get the current angle of the arm joint
   *
   * @return Current angle
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(inputs.angleDegrees);
  }

  /**
   * Get the current velocity of the arm joint
   *
   * @return Velocity, in degrees per seconds
   */
  public double getVelocityDps() {
    return inputs.leftVelocityDps;
  }

  /**
   * Get the target arm joint angle
   *
   * @return Target angle
   */
  public Rotation2d getTargetAngle() {
    return targetRotation;
  }

  public Command setTargetAngleCommand(Rotation2d angle) {
    return run(() -> setTargetAngle(angle));
  }

  public Command setVolt(double voltage) {
    return run(() -> setManual(voltage));
  }

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              io.optimizeForSysID();
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/ArmJoint");
              }
              SignalLogger.start();
            }),
        sysIdRoutine.quasistatic(direction));
  }

  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              io.optimizeForSysID();
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/ArmJoint");
              }
              SignalLogger.start();
            }),
        sysIdRoutine.dynamic(direction));
  }
}
