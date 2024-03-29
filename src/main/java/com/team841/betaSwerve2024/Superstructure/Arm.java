package com.team841.betaSwerve2024.Superstructure;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.SC;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  final TalonFX leftArmJoint = new TalonFX(ConstantsIO.CANID.kLeftArmJoint, "rio");

  final TalonFX rightArmJoint = new TalonFX(ConstantsIO.CANID.kRightArmJoint, "rio");

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

  public void hardStop() {
    this.rightArmJoint.setControl(new StaticBrake());
  }

  @Override
  public void periodic() {}
}
