package com.team841.betaSwerve2024.Drive.SysID;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.team841.betaSwerve2024.Constants.SubsystemManifest;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysID {

  private static SysIdRoutine DriveRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, null, null, (state) -> SignalLogger.writeString("State", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) ->
                  SubsystemManifest.drivetrain.setControl(
                      new DriveVoltageRequest().withVoltage(volts.in(Volts))),
              null,
              SubsystemManifest.drivetrain));

  public static Command DrivesysIdQuasistatic(SysIdRoutine.Direction direction) {
    return DriveRoutine.quasistatic(direction);
  }

  public static Command DrivesysIdDynamic(SysIdRoutine.Direction direction) {
    return DriveRoutine.dynamic(direction);
  }

  private static SysIdRoutine SteerRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, null, null, (state) -> SignalLogger.writeString("State", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) ->
                  SubsystemManifest.drivetrain.setControl(
                      new AzimuthVoltageRequest().withVoltage(volts.in(Volts))),
              null,
              SubsystemManifest.drivetrain));

  public static Command SteersysIdQuasistatic(SysIdRoutine.Direction direction) {
    return SteerRoutine.quasistatic(direction);
  }

  public static Command SteersysIdDynamic(SysIdRoutine.Direction direction) {
    return SteerRoutine.dynamic(direction);
  }

  public static Command doNothing() {
    return new RunCommand(
        () -> {
          SubsystemManifest.drivetrain.setControl(new SwerveRequest.Idle());
        },
        SubsystemManifest.drivetrain);
  }
}
