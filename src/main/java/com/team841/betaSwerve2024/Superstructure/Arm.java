package com.team841.betaSwerve2024.Superstructure;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.betaSwerve2024.Constants.SC;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  final TalonFX leftArmJoint = new TalonFX(13, "rio");

  final TalonFX rightArmJoint = new TalonFX(14, "rio");

  public Arm() {
    leftArmJoint.getConfigurator().apply(SC.Arm.k_ArmConfiguration);
    rightArmJoint.getConfigurator().apply(SC.Arm.k_ArmConfiguration);
    leftArmJoint.setControl(new Follower(rightArmJoint.getDeviceID(), true));
  }

  public void forward() {
    this.rightArmJoint.setControl(new DutyCycleOut(-0.1));
  }

  public void backward() {
    this.rightArmJoint.setControl(new DutyCycleOut(0.1));
  }

  public void stop() {
    this.rightArmJoint.stopMotor();
  }

  @Override
  public void periodic() {}
}
