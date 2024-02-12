package com.team841.betaSwerve2024.Superstructure;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.betaSwerve2024.Constants.SC;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {

  private final TalonFX kickerTalon = new TalonFX(10, "rio");

  private DigitalInput kickerSensor = new DigitalInput(SC.Kicker.k_kickerSensorChannel);

  public Kicker() {
    kickerTalon.getConfigurator().apply(SC.Kicker.k_KickerConfiguration);
  }

  private void setDutyCyle(double speed) {
    kickerTalon.setControl(new DutyCycleOut(speed));
  }

  public void intake() {
    setDutyCyle(-0.6);
  }

  public void stopKicker() {
    kickerTalon.stopMotor();
  }

  public void Pass() {
    setDutyCyle(-0.6);
  }

  public boolean getKickerSensor() {
    return !(kickerSensor.get());
  }

  @Override
  public void periodic() {}
}
