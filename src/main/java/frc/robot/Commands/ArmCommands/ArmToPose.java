// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Util.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Vision.AprilTagLocalizer;


public class ArmToPose extends Command {
  /** Creates a new ArmToPose. */
  ArmSubsystem m_arm;
  AprilTagLocalizer m_tag;
  double angle = 0.0;

  public ArmToPose(ArmSubsystem m_arm, AprilTagLocalizer m_tag) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_tag = m_tag;

    addRequirements(m_arm);
  }

  public ArmToPose(ArmSubsystem m_arm2, Object object) {
    //TODO Auto-generated constructor stub
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_arm.getController().reset(m_arm.getArmAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        if(m_tag.hasTargets()){
         angle = m_arm.getAngleForDistance(m_tag.getDistance());
        }
        else{
            angle = 160;
        }
     m_arm.updateArmSetpoint(angle);
      } 
      
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.updateArmSetpoint(160.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}