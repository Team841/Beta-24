package com.team841.betaSwerve2024;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.SubsystemManifest;
import com.team841.betaSwerve2024.Constants.Swerve;
import com.team841.betaSwerve2024.Constants.TunerConstants;
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
  private double MaxSpeed = Swerve.kSpeedAt12VoltsMps; // 6 meters per second desired top speed
  private double MaxAngularRate = 4 * Math.PI;
      // 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandPS5Controller joystick =
      new CommandPS5Controller(ConstantsIO.OI.driverPortLeft); // My joystick
  private final CommandXboxController cojoystick =
      new CommandXboxController(ConstantsIO.OI.codriverPort);
  private final Drivetrain drivetrain = SubsystemManifest.drivetrain; // My drivetrain
  private final Intake intake = SubsystemManifest.intake;

  private final Indexer Indexer = SubsystemManifest.indexer;

  private final Shooter shooter = SubsystemManifest.shooter;

  private final Arm arm = SubsystemManifest.arm;

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

    // joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.circle().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.touchpad().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    } else {
      drivetrain.registerTelemetry(logger::telemeterize);
    }
  }

  // xbox
  public void configureCoBindings() {

    Command c_command = new IntakeCommand(intake, Indexer);
    /* cojoystick.leftBumper().whileTrue(c_command);
    cojoystick
        .leftTrigger()
        .onTrue(new InstantCommand(shooter::spinUp))
        .onFalse(new InstantCommand(shooter::stopShooter));
    cojoystick
        .rightTrigger()
        .onTrue(new InstantCommand(Indexer::Pass))
        .onFalse(new InstantCommand(Indexer::stopIndexer));
    cojoystick
        .rightBumper()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(Indexer::stopIndexer),
                new InstantCommand(shooter::stopShooter)));
    cojoystick.x().onTrue(new InstantCommand(shooter::ampShot)).onFalse(new InstantCommand(shooter::stopShooter));

    cojoystick.a().whileTrue(new InstantCommand(arm::forward)).onFalse(new InstantCommand(arm::stop));
    cojoystick.y().whileTrue(new InstantCommand(arm::backward)).onFalse(new InstantCommand(arm::stop));
    */
    joystick.L1().whileTrue(c_command);
    joystick
            .L2()
            .onTrue(new InstantCommand(shooter::spinUp))
            .onFalse(new InstantCommand(shooter::stopShooter));
    joystick
            .R2()
            .onTrue(new InstantCommand(Indexer::Pass))
            .onFalse(new InstantCommand(Indexer::stopIndexer));
    joystick
            .R1()
            .onTrue(
                    new SequentialCommandGroup(
                            new InstantCommand(Indexer::stopIndexer),
                            new InstantCommand(shooter::stopShooter)));
    joystick.square().onTrue(new InstantCommand(shooter::ampShot)).onFalse(new InstantCommand(shooter::stopShooter));

    joystick.cross().whileTrue(new InstantCommand(arm::forward)).onFalse(new InstantCommand(arm::stop));
    joystick.triangle().whileTrue(new InstantCommand(arm::backward)).onFalse(new InstantCommand(arm::stop));
  }

  public RobotContainer() {
    // Register Named Commands
    NamedCommands.registerCommand("IntakeOn", new IntakeCommand(intake, Indexer));
    NamedCommands.registerCommand("Shoot", new ParallelCommandGroup(new InstantCommand(shooter::spinUp), new SequentialCommandGroup(new WaitCommand(1), new InstantCommand(Indexer::Pass))).withTimeout(3));
    NamedCommands.registerCommand("SpinUp", new InstantCommand(shooter::spinUp));
    NamedCommands.registerCommand("JustShoot", new InstantCommand(Indexer::Pass));
    NamedCommands.registerCommand("ALLSYSTEMSGO", new ParallelCommandGroup(new InstantCommand(intake::intake), new InstantCommand(shooter::spinUp), new InstantCommand(Indexer::Pass)));
    NamedCommands.registerCommand("JustStop", new ParallelCommandGroup(new InstantCommand(Indexer::stopIndexer), new InstantCommand(shooter::stopShooter)));

    configureBindings();
    configureCoBindings();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    // auto chooser on shuffleboard
    return autoChooser.getSelected();
  }
}
