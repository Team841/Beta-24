package com.team841.betaSwerve2024.Superstructure;

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
  public void execute() {
    if (f_indexer.getindexerSensor() && !(f_indexer.getLeftIndexerSensor())) {
      f_indexer.halfIntake();
    } else if (f_indexer.getLeftIndexerSensor() && !(f_indexer.getindexerSensor())) {
      f_indexer.halfIntake();
    } else if (!(f_indexer.getLeftIndexerSensor()) && !(f_indexer.getindexerSensor())) {
      f_indexer.intake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    f_intake.stopIntake();
    f_indexer.stopIndexer();
  }

  @Override
  public boolean isFinished() {
    return f_indexer.getindexerSensor() && f_indexer.getLeftIndexerSensor();
  }
}
