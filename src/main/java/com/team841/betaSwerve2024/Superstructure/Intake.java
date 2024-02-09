package com.team841.betaSwerve2024.Superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team841.betaSwerve2024.Constants.SC;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeOneMotor = new CANSparkMax(ConstantsIO.CANID.kIntakeOne, MotorType.kBrushless);

  private final DigitalInput Intake_Index_Sensor = new DigitalInput(0);

  private int outTakeClock = 0;

  public Intake() {

    intakeOneMotor.restoreFactoryDefaults();

    intakeOneMotor.setSmartCurrentLimit(SC.Intake.currentLimit);

    setIntakeBrakes(true);
  }

  public void setIntakeBrakes(boolean on) {
    intakeOneMotor.setIdleMode(on ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  public void setIntakeMotor(Double speed) {
    intakeOneMotor.set(speed);
  }

  public void intake() {
    setIntakeMotor(1.0);
  }

  public void outTake() {
    setIntakeMotor(-1.0);
  }

  public void StopTake() {
    setIntakeMotor(0.0);
  }

  public Command toggleIn() {
    return new ConditionalCommand(new InstantCommand(this::StopTake), new InstantCommand(this::intake),
        () -> intakeOneMotor.get() > 0);
  }

  public Command toggleOut() {
    return new ConditionalCommand(new InstantCommand(this::StopTake), new InstantCommand(this::outTake),
        () -> intakeOneMotor.get() < 0);
  }

  public boolean getSensor() {
    return !Intake_Index_Sensor.get();
  }

  @Override
  public void periodic() {
  }
}