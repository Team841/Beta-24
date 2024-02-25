package com.team841.betaSwerve2024.Superstructure;

import com.team841.betaSwerve2024.Constants.Manifest;
import com.team841.betaSwerve2024.RobotContainer;
import com.team841.betaSwerve2024.Rumble;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

  private Intake f_intake;
  private Indexer f_indexer;

  public IntakeCommand(Intake omg_Intake, Indexer omg_indexer) {
    this.f_intake = omg_Intake;
    this.f_indexer = omg_indexer;
    addRequirements(f_intake, f_indexer);
  }

  @Override
  public void initialize() {
    f_intake.intake();
    f_indexer.intake();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    f_intake.stopIntake();
    f_indexer.stopIndexer();
    f_intake.resetStop();
  }

  @Override
  public boolean isFinished() {
    return f_indexer.getindexerSensor();
  }
}
