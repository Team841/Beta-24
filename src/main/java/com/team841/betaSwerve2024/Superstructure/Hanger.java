// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team841.betaSwerve2024.Superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team841.betaSwerve2024.Constants.*;
public class Hanger extends SubsystemBase {
  private CANSparkMax LeftHangerMotor;
  private CANSparkMax RightHangerMotor;
  /** Creates a new Hanger. */
  public Hanger() {
    LeftHangerMotor = new CANSparkMax(ConstantsIO.CANID.kHangerMoterLeft, MotorType.kBrushed);
    RightHangerMotor = new CANSparkMax(ConstantsIO.CANID.kHangerMotorRight, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void ExtendHanger() {
    LeftHangerMotor.set(.25);
    RightHangerMotor.set(.25);
  }
  public void RetractHanger() {
    LeftHangerMotor.set(-.25);
    RightHangerMotor.set(-.25);
  }
  public void StopHanger() {
    LeftHangerMotor.set(0);
    RightHangerMotor.set(0);
  }
}
