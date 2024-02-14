// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team841.betaSwerve2024.Superstructure.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team841.betaSwerve2024.Superstructure.Intake;

public class IntakeStop extends Command {
  /** Creates a new IntakeStop. */
  private final Intake cmd_intake;

  public IntakeStop(Intake subsystem) {
    cmd_intake = subsystem;
    addRequirements(cmd_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd_intake.stopTake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
