package com.team841.betaSwerve2024.Superstructure;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.betaSwerve2024.Constants.SC;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private final TalonFX indexerTalon = new TalonFX(10, "rio");

  private DigitalInput indexerSensor = new DigitalInput(SC.Indexer.k_IndexerSensorChannel);

  public Indexer() {
    indexerTalon.getConfigurator().apply(SC.Indexer.k_IndexerConfiguration);
  }

  private void setDutyCyle(double speed) {
    indexerTalon.setControl(new DutyCycleOut(speed));
  }

  public void intake() {
    setDutyCyle(-0.6);
  }

  public void stopIndexer() {
    indexerTalon.stopMotor();
  }

  public void Pass() {
    setDutyCyle(-0.6);
  }

  public boolean getindexerSensor() {
    return !(indexerSensor.get());
  }

  @Override
  public void periodic() {}
}
