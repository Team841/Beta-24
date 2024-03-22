package com.team841.betaSwerve2024.Superstructure;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.SC;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private final TalonFX indexerTalon = new TalonFX(ConstantsIO.CANID.kIndexerTalon, "rio");

  private DigitalInput indexerSensor = new DigitalInput(SC.Indexer.k_IndexerSensorChannel);
  private DigitalInput leftIndexerSensor = new DigitalInput(3);

  public Indexer() {
    indexerTalon.getConfigurator().apply(SC.Indexer.k_IndexerConfiguration);
  }

  private void setDutyCyle(double speed) {
    indexerTalon.setControl(new DutyCycleOut(speed));
  }

  public void intake() {
    setDutyCyle(-0.6);
  }

  public void halfIntake() {
    setDutyCyle(-0.3);
  }

  public void quarterIntake(){
    setDutyCyle(-0.15);
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

  public boolean getLeftIndexerSensor() {
    return !(leftIndexerSensor.get());
  }

  public void reverseIndexer() {
    setDutyCyle(0.6);
  }

  @Override
  public void periodic() {}
}
