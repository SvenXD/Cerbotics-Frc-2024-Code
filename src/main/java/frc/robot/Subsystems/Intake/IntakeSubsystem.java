package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.Util.Logging.LoggedTunableNumber;

import static frc.robot.Constants.Arm.*;

public class IntakeSubsystem extends SubsystemBase {

  /*Io and inputs */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  
  /* PID Gains */
  LoggedTunableNumber armKg = new LoggedTunableNumber("ArmPID/kG", kG);
  LoggedTunableNumber armKs = new LoggedTunableNumber("ArmPID/kS", kS);
  LoggedTunableNumber armKa = new LoggedTunableNumber("ArmPID/kA", kA);
  LoggedTunableNumber armKv = new LoggedTunableNumber("ArmPID/kV", kV);
  LoggedTunableNumber armKp = new LoggedTunableNumber("ArmPID/kP", kP);
  LoggedTunableNumber armKi = new LoggedTunableNumber("ArmPID/kI", kI);
  LoggedTunableNumber armKd = new LoggedTunableNumber("ArmPID/kD", kD);


  /* Constrains */
    private final TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_tpState = new TrapezoidProfile.State(0.0, 0.0);
    private ProfiledPIDController m_controller;

    private final ArmFeedforward m_feedforward;

  /*Variables */
    private SendableChooser<String> armModeChooser = new SendableChooser<>();
    private String currentModeSelection;
    private final String[] modeNames = {"BRAKE", "COAST"};
    private Boolean enable = false;

    public static enum ArmStates{
      INTAKING,
      FEEDING,
      STANDING,
      SHOOTING,
      IDLE
    }

    private ArmStates systemStates = ArmStates.IDLE;

    
  static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> 
    kDistanceToArmAngle = new InterpolatingTreeMap<>();

  static{ //Added offset of 0 degrees
    kDistanceToArmAngle.put(new InterpolatingDouble(1.66),  new InterpolatingDouble(160.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(2.05),  new InterpolatingDouble(153.0)); 
    kDistanceToArmAngle.put(new InterpolatingDouble(2.66),  new InterpolatingDouble(143.5));
    kDistanceToArmAngle.put(new InterpolatingDouble(3.50),  new InterpolatingDouble(138.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(4.15),  new InterpolatingDouble(135.0));
    kDistanceToArmAngle.put(new InterpolatingDouble(4.35),  new InterpolatingDouble(134.0));
  }

  
  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    Logger.processInputs("Arm", inputs);

    m_feedforward = new ArmFeedforward(armKs.get(), armKg.get(), armKv.get(), armKa.get());
    m_constraints = new TrapezoidProfile.Constraints(kMaxVelocityRadPerSecond,kMaxAccelerationMetersPerSecondSquared);
    m_controller = new ProfiledPIDController(armKp.get(),armKi.get(),armKd.get(),m_constraints,kPeriod);



    armModeChooser.setDefaultOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Brake Mode", modeNames[0]);
    armModeChooser.addOption("Coast Mode", modeNames[1]);

    SmartDashboard.putData("Arm Mode", armModeChooser);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updatePID();

    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/Current State", systemStates.toString());

    if(enable){
     io.setIntakePosition(m_controller.calculate(inputs.positionDegrees), 
     m_feedforward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity));
    }
  
      
  }

  
    public Command goToPosition(double position){
    Command ejecutable = Commands.runOnce(
                () -> {
                getController().reset(inputs.positionDegrees);
                m_controller.setGoal(position);
                enable = true;
                },
                this);
    return ejecutable;
  }

  public double getAngleForDistance(double distance){
    return kDistanceToArmAngle.getInterpolated(
      new InterpolatingDouble(
        Math.max(Math.min(distance, 4.35 ), 1.66))).value;
  }
  
  public ProfiledPIDController getController(){
    return m_controller;
  }
     
  public void updatePID(){
    if (armKs.hasChanged(0)
      || armKv.hasChanged(0)
      || armKp.hasChanged(0)
      || armKi.hasChanged(0)
      || armKd.hasChanged(0)
      || armKa.hasChanged(0)) {
    
        m_controller = new ProfiledPIDController(
        armKp.get(), 
        armKi.get(), 
        armKd.get(), 
        m_constraints);
    }
  }

  public double getAngleDegrees(){
    return inputs.positionDegrees;
  }
  


  public ArmStates getState(){
    return systemStates;
  }

  public ArmStates changeState(ArmStates state){
    systemStates = state;
    return systemStates;
  }

  public void updateArmSetpoint(double setpoint){
    m_tpState.position = Units.degreesToRadians(setpoint);
    m_controller.setGoal(setpoint);
  }

}