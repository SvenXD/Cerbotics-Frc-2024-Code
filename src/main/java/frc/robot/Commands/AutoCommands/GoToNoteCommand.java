// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Commands.SwerveCommands.NoteAlignCommand;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Swerve.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class GoToNoteCommand extends ParallelRaceGroup {

    Drive drive;


    /** Creates a new autoCollectNote. */
    public GoToNoteCommand(Drive drive) {
        this.drive = drive;
      
        addCommands(new NoteAlignCommand(drive).withTimeout(2.50));
    }
}