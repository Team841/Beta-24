// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team841.betaSwerve2024.Superstructure;

import com.team841.betaSwerve2024.Constants.Manifest;
import com.team841.betaSwerve2024.Constants.SC;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private final Spark LED = new Spark(SC.Intake.kBlinkingID);

  private Indexer indexer = Manifest.SubsystemManifest.indexer;

  private Intake intake = Manifest.SubsystemManifest.intake;

  private int msCount = 0;

  /** Creates a new LED. */
  public LED() {}

  // public void setColor(String color) {
  //   switch (color) {
  //     case "Violet" -> LED.set(0.91);
  //     case "Green" -> LED.set(.77);
  //     case "Orange" -> LED.set(.65);
  //   }
  // }

  @Override
  public void periodic() {
    // if (intake.)
    
    if (indexer.getindexerSensor() && indexer.getLeftIndexerSensor()) {
      msCount = 0;
    }
    
    if(msCount < 3000){ 
      //green
      msCount += 20;
      LED.set(0.71);
    }
    else {
      //red
      LED.set(0.59);
    }
  }
}
