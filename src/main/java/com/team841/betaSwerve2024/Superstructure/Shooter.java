package com.team841.betaSwerve2024.Superstructure;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.betaSwerve2024.Constants.SC;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final TalonFX bottomShooter = new TalonFX(12, "rio");
  private final TalonFX topShooter = new TalonFX(11, "rio");

  public Shooter() {
    bottomShooter.getConfigurator().apply(SC.Shooter.k_BottomConfiguration);
    topShooter.getConfigurator().apply(SC.Shooter.k_TopConfiguration);
    // bottomShooter.setControl(new Follower(topShooter.getDeviceID(), false));
  }

  private void setVelocity(double velocity) {
    topShooter.setControl(new MotionMagicVelocityVoltage(velocity));
  }

  protected void Shoot() {
    setVelocity(60);
  }

  public void spinUp() {
    topShooter.setControl(
        new MotionMagicVelocityVoltage(95).withFeedForward(6).withAcceleration(200).withSlot(0));
    bottomShooter.setControl(
        new MotionMagicVelocityVoltage(95).withFeedForward(6).withAcceleration(200).withSlot(0));
  }

  public void ampShot() {
    topShooter.setControl(
        new MotionMagicVelocityVoltage(2).withFeedForward(6).withAcceleration(200).withSlot(0));
    bottomShooter.setControl(
        new MotionMagicVelocityVoltage(20).withFeedForward(6).withAcceleration(200).withSlot(0));
  }

  protected double getMotorVoltage() {
    return topShooter.getMotorVoltage().getValue();
  }

  public boolean isShooting() {
    return this.topShooter.getVelocity().getValue() > 0;
  }

  public void stopShooter() {
    topShooter.stopMotor();
    bottomShooter.stopMotor();
  }

  public Command runShooter(double velocity) {
    return new InstantCommand(() -> this.setVelocity(velocity));
  }

  @Override
  public void periodic() {}
}
