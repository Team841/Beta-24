package com.team841.betaSwerve2024;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.Manifest;
import com.team841.betaSwerve2024.Constants.Swerve;
import com.team841.betaSwerve2024.Drive.AutoShoot;
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

  //public final CommandXboxController joystick = Manifest.JoystickManifest.joystick; // My joystick
  //public final CommandXboxController cojoystick = Manifest.JoystickManifest.cojoystick;
  public final CommandPS5Controller duoStickDrive = Manifest.JoystickManifest.joystick;
  public final CommandXboxController duoStickCoDrive = Manifest.JoystickManifest.cojoystick;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final Drivetrain drivetrain = Manifest.SubsystemManifest.drivetrain; // My drivetrain
  private final Intake intake = Manifest.SubsystemManifest.intake;

  private final Indexer indexer = Manifest.SubsystemManifest.indexer;

  private final Shooter shooter = Manifest.SubsystemManifest.shooter;

  private final Arm arm = Manifest.SubsystemManifest.arm;

  private final LED led = Manifest.SubsystemManifest.led;

  private final Hanger hanger = Manifest.SubsystemManifest.hanger;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Swerve.MaxSpeed * 0.1)
          .withRotationalDeadband(Swerve.MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final AutoShoot autoAim = new AutoShoot(drivetrain, indexer, shooter);
  private final Telemetry logger = new Telemetry(Swerve.MaxSpeed);

  private final SendableChooser<Command> autoChooser;

  private final Command BackOffTrap = AutoBuilder.buildAuto("Drive Off");

  private void configureDuoStick() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                    () ->
                            drive
                                    .withVelocityX(-duoStickDrive.getLeftY() * Swerve.MaxSpeed) // Drive forward with
                                    // negative Y (forward)
                                    .withVelocityY(
                                            -duoStickDrive.getLeftX() * Swerve.MaxSpeed) // Drive left with negative X (left)
                                    .withRotationalRate(
                                            -duoStickDrive.getRightX()
                                                    * Swerve
                                                    .MaxAngularRate))); // Drive counterclockwise with negative X (left)


    duoStickDrive.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    duoStickDrive
            .circle()
            .whileTrue(
                    drivetrain.applyRequest(
                            () ->
                                    point.withModuleDirection(
                                            new Rotation2d(-duoStickDrive.getLeftY(), -duoStickDrive.getLeftX()))));

    // reset the field-centric heading on left bumper press
    duoStickDrive.touchpad().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //duoStickDrive.R2().whileTrue(autoAim);

    //duoStickDrive.triangle().whileTrue(BackOffTrap);

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    } else {
      drivetrain.registerTelemetry(logger::telemeterize);
    }

    Command c_command = new IntakeCommand(intake, indexer);
    duoStickCoDrive.leftBumper().whileTrue(c_command);
    duoStickCoDrive
            .leftTrigger()
            .onTrue(new InstantCommand(shooter::spinUp))
            .onFalse(new InstantCommand(shooter::stopShooter));
    duoStickCoDrive
            .rightTrigger()
            .onTrue(
                    new ConditionalCommand(
                            new InstantCommand(indexer::Pass),
                            new InstantCommand(indexer::stopIndexer),
                            () -> shooter.isShooting()))
            .onFalse(new InstantCommand(indexer::stopIndexer));
    duoStickCoDrive
            .rightBumper()
            .onTrue(
                    new SequentialCommandGroup(
                            new InstantCommand(indexer::stopIndexer),
                            new InstantCommand(shooter::stopShooter)));
    duoStickCoDrive.povUp().whileTrue(new InstantCommand(hanger::ExtendHanger)).onFalse(new InstantCommand(hanger::StopHanger));
    duoStickCoDrive.povDown().whileTrue(new InstantCommand(hanger::RetractHanger)).onFalse(new InstantCommand(hanger::StopHanger));
    duoStickCoDrive.povCenter().whileTrue(new InstantCommand(hanger::StopHanger));
    //duoStickCoDrive.povLeft().whileTrue(new InstantCommand(hanger::toggleHanger));
    duoStickCoDrive
            .x()
            .onTrue(new InstantCommand(shooter::ampShot))
            .onFalse(new InstantCommand(shooter::stopShooter));
    duoStickCoDrive
            .b()
            .onTrue(
                    new ParallelCommandGroup(
                            new InstantCommand(intake::outTake), new InstantCommand(indexer::reverseIndexer)))
            .onFalse(
                    new SequentialCommandGroup(
                            new InstantCommand(indexer::stopIndexer), new InstantCommand(intake::stopIntake)));

    duoStickCoDrive
            .y()
            .whileTrue(new InstantCommand(shooter::flyShot))
            .onFalse(new InstantCommand(shooter::stopShooter));
    duoStickCoDrive
            .a()
            .onTrue(new InstantCommand(shooter::trapShot))
            .onFalse(new InstantCommand(shooter::stopShooter));
  }

   public RobotContainer() {
    // Register Named Commands
    NamedCommands.registerCommand("IntakeOn", new IntakeAuto(intake, indexer));
    NamedCommands.registerCommand(
        "Shoot",
        new ParallelCommandGroup(
                new InstantCommand(shooter::spinUp),
                new SequentialCommandGroup(new WaitCommand(1), new InstantCommand(indexer::Pass)))
            .withTimeout(3));
    NamedCommands.registerCommand("SpinUp", new InstantCommand(shooter::spinUp));
    NamedCommands.registerCommand("JustShoot", new InstantCommand(indexer::Pass).withTimeout(0.5));
    NamedCommands.registerCommand(
        "ALLSYSTEMSGO",
        new ParallelCommandGroup(
                new InstantCommand(intake::intake),
                new InstantCommand(shooter::disruptshot),
                new InstantCommand(indexer::Pass)));
    NamedCommands.registerCommand(
        "FunnyInake",
        new ParallelCommandGroup(
                new InstantCommand(intake::intake), new InstantCommand(indexer::Pass))
            .withTimeout(0.75));
    NamedCommands.registerCommand(
        "JustStop",
        new ParallelCommandGroup(
            new InstantCommand(indexer::stopIndexer), new InstantCommand(shooter::stopShooter)));
    NamedCommands.registerCommand(
        "sam",
        new ParallelCommandGroup(
                new InstantCommand(intake::intake),
                new InstantCommand(shooter::ampShot),
                new InstantCommand(indexer::Pass))
            .withTimeout(2.5));
    NamedCommands.registerCommand("aIntake", new IntakeAuto(intake, indexer));
    NamedCommands.registerCommand("stopIndexer", new InstantCommand(indexer::stopIndexer));
    NamedCommands.registerCommand(
        "CountShot",
        new ParallelCommandGroup(
                new InstantCommand(shooter::spinUp),
                new SequentialCommandGroup(new WaitCommand(1), new InstantCommand(indexer::Pass)))
            .withTimeout(0.8));
    NamedCommands.registerCommand("STOPALL", new ParallelCommandGroup(
            new InstantCommand(indexer::stopIndexer), new InstantCommand(shooter::stopShooter), new InstantCommand(intake::stopIntake)));

    configureDuoStick();
    //configureBindings();
    //configureCoBindings();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    // auto chooser on shuffleboard
    return autoChooser.getSelected();
  }
}
