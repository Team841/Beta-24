// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team841.betaSwerve2024.Superstructure;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team841.betaSwerve2024.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hanger extends SubsystemBase {
  private TalonFX LeftHangerMotor = new TalonFX(ConstantsIO.CANID.kHangerMotorLeft, "rio");
  private TalonFX RightHangerMotor = new TalonFX(ConstantsIO.CANID.kHangerMotorRight, "rio");

  private boolean FlipHanger = true;

  /** Creates a new Hanger. */
  public Hanger() {

    LeftHangerMotor.getConfigurator()
        .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    RightHangerMotor.getConfigurator()
        .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ExtendHanger() {
    if (FlipHanger) {
      LeftHangerMotor.set(-1.00);
      RightHangerMotor.set(-1.00);
    } else {
      LeftHangerMotor.set(1.00);
      RightHangerMotor.set(1.00);
    }
  }

  public void RetractHanger() {
    if (FlipHanger) {
      LeftHangerMotor.set(.98);
      RightHangerMotor.set(.98);
    } else {
      LeftHangerMotor.set(-.98);
      RightHangerMotor.set(-.98);
    }
  }

  public void toggleHanger() {
    FlipHanger = FlipHanger ? false : true;
  }

  public void StopHanger() {
    LeftHangerMotor.set(0);
    RightHangerMotor.set(0);
  }
}
