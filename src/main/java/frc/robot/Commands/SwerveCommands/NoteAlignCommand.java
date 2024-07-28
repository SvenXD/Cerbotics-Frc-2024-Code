package frc.robot.Commands.SwerveCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Util.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Drive;

public class NoteAlignCommand extends Command {

  Drive m_drive;
  double tx = 0;
  double omegaSpeed;

  boolean m_isFinished;

  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(.1, .01);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(.095, 0, 0, OMEGA_CONSTRATINTS);
  public NoteAlignCommand(Drive m_drive) {
    this.m_drive = m_drive;

    omegaController.setTolerance(1);
    omegaController.setGoal(0);
    addRequirements(m_drive);
  }


  @Override
  public void initialize() {

      System.out.println("Starting driveToNote");
            SmartDashboard.putData("Note Detect PID",omegaController);

        omegaController.reset(tx);
        if (LimelightHelpers.getTV(VisionConstants.neuralLimelight)) {
            m_isFinished = false;

        } else {
            if (DriverStation.isAutonomous()) {
                m_isFinished = true;

            } else {
                m_isFinished = false;
            }  
        }        
  }


  @Override
  public void execute() {
    tx = LimelightHelpers.getTX(VisionConstants.neuralLimelight);
    omegaSpeed = omegaController.calculate(tx);
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    -1* (1 - Math.abs(tx) / 32) * .50,
    0,
    -omegaSpeed, 
    m_drive.getRotation());

    if (omegaController.atGoal()) {
        omegaSpeed = 0;
    }

    if (LimelightHelpers.getTV(VisionConstants.neuralLimelight)) {

      m_drive.runVelocity(chassisSpeeds);
            }
    
    else {
      m_drive.runVelocity(new ChassisSpeeds(0,0,0));
    }
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
