// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team841.betaSwerve2024.Superstructure;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final Spark LED = new Spark(4);
  /** Creates a new LED. */

  public LED() {}
  public void setColor (String color){
    if (color == "Yellow"){
      LED.set(0.69);
    }
    else if (color == "Violet"){
      LED.set(0.91);
    }
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
