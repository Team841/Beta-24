package com.team841.betaSwerve2024.Superstructure;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.SC;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeOneMotor =
      new CANSparkMax(ConstantsIO.CANID.kIntake, MotorType.kBrushless);

  public Intake() {

    intakeOneMotor.restoreFactoryDefaults();

    intakeOneMotor.setSmartCurrentLimit(SC.Intake.kCurrentLimit);

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
  }

  public void stopIntake() {
    setIntakeMotor(0.0);
  }

  @Override
  public void periodic() {}
}
