package com.team841.betaSwerve2024.Superstructure;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX intakeOneMotor = new TalonFX(ConstantsIO.CANID.kIntake, "rio");

  public Intake() {

    setIntakeBrakes(true);
  }

  public void setIntakeBrakes(boolean on) {
    intakeOneMotor
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(on ? NeutralModeValue.Brake : NeutralModeValue.Coast));
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
