// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team841.betaSwerve2024.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;

public class UpdateLED extends Command {
  private LED f_led;
  private Indexer f_indexer;

  /** Creates a new UpdateLED. */
  public UpdateLED(LED led, Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.f_led = led;
    this.f_indexer = indexer;
    addRequirements(f_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (f_indexer.getindexerSensor()) {
      f_led.setColor("Green");
    } else {
      f_led.setColor("Violet");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
