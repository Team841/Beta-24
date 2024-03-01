package com.team841.betaSwerve2024.Superstructure;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.SC;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeOneMotor =
      new CANSparkMax(ConstantsIO.CANID.kIntakeOne, MotorType.kBrushless);

  private boolean stop;

  private int outTakeClock = 0;

  private String toggle = "OFF";

  public Intake() {

    intakeOneMotor.restoreFactoryDefaults();

    intakeOneMotor.setSmartCurrentLimit(SC.Intake.currentLimit);

    setIntakeBrakes(true);
  }

  public void setIntakeBrakes(boolean on) {
    intakeOneMotor.setIdleMode(on ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  public void setIntakeMotor(Double speed) {
    intakeOneMotor.set(speed);
  }

  public void intake() {
    setIntakeMotor(1.0);
  }

  public void outTake() {
    setIntakeMotor(-1.0);
    toggle = "OUT";
  }

  public void stopIntake() {
    setIntakeMotor(0.0);
    toggle = "OFF";
  }

  public String getCurrentState() {
    return toggle;
  }

  public void setStop() {
    stop = true;
  }

  public void resetStop() {
    stop = false;
  }

  public Command toggleIn() {
    return new ConditionalCommand(
        new InstantCommand(this::stopIntake),
        new InstantCommand(this::intake),
        () -> intakeOneMotor.get() > 0);
  }

  public Command toggleOut() {
    return new ConditionalCommand(
        new InstantCommand(this::stopIntake),
        new InstantCommand(this::outTake),
        () -> intakeOneMotor.get() < 0);
  }

  @Override
  public void periodic() {}
}
