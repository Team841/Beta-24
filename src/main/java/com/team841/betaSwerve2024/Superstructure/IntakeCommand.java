package com.team841.betaSwerve2024.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

  private Intake f_intake;
  private Kicker f_kicker;

  public IntakeCommand(Intake omg_Intake, Kicker omg_kicker) {
    this.f_intake = omg_Intake;
    this.f_kicker = omg_kicker;
    addRequirements(f_intake, f_kicker);
  }

  @Override
  public void initialize() {
    f_intake.intake();
    f_kicker.intake();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    f_intake.stopIntake();
    f_kicker.stopKicker();
    f_intake.resetStop();
  }

  @Override
  public boolean isFinished() {
    return f_kicker.getKickerSensor();
  }
}
