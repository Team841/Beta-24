package com.team841.betaSwerve2024;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.SubsystemManifest;
import com.team841.betaSwerve2024.Drive.Drivetrain;
import com.team841.betaSwerve2024.Superstructure.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandPS5Controller joystick =
      new CommandPS5Controller(ConstantsIO.OI.driverPortLeft); // My joystick
  private final CommandXboxController cojoystick =
      new CommandXboxController(ConstantsIO.OI.codriverPort);
  private final Drivetrain drivetrain = SubsystemManifest.drivetrain; // My drivetrain
  private final Intake intake = SubsystemManifest.intake;

  private final Kicker kicker = SubsystemManifest.kicker;

  private final Shooter shooter = SubsystemManifest.shooter;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> autoChooser;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .circle()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.L1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    } else {
      drivetrain.registerTelemetry(logger::telemeterize);
    }
  }

  public void justStopStop() {}

  // xbox
  public void configureCoBindings() {

    Command c_command = new IntakeCommand(intake, kicker);
    cojoystick.leftBumper().whileTrue(c_command);
    cojoystick
        .leftTrigger()
        .onTrue(new InstantCommand(shooter::spinUp))
        .onFalse(new InstantCommand(shooter::stopShooter));
    cojoystick
        .rightTrigger()
        .onTrue(new InstantCommand(kicker::Pass))
        .onFalse(new InstantCommand(kicker::stopKicker));
    cojoystick
        .rightBumper()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(kicker::stopKicker), new InstantCommand(shooter::stopShooter)));
  }

  public RobotContainer() {
    configureBindings();
    configureCoBindings();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
