package com.team841.betaSwerve2024;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team841.betaSwerve2024.Autonomous.Autos;
import com.team841.betaSwerve2024.Constants.Manifest;
import com.team841.betaSwerve2024.Constants.Swerve;
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

  public final CommandPS5Controller joystick = Manifest.JoystickManifest.joystick; // My joystick
  public final CommandXboxController cojoystick = Manifest.JoystickManifest.cojoystick;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final Drivetrain drivetrain = Manifest.SubsystemManifest.drivetrain; // My drivetrain
  private final Intake intake = Manifest.SubsystemManifest.intake;

  private final Indexer indexer = Manifest.SubsystemManifest.indexer;

  private final Shooter shooter = Manifest.SubsystemManifest.shooter;

  private final Arm arm = Manifest.SubsystemManifest.arm;

  private final LED led = Manifest.SubsystemManifest.led;

  private final Hanger hanger = Manifest.SubsystemManifest.hanger;

  // driving in open loop
  private final SwerveRequest.FieldCentric GenericFieldCentricControl =
      new SwerveRequest.FieldCentric()
          .withDeadband(Swerve.Controls.kDriveDeadBand) // Add a 10% deadband
          .withRotationalDeadband(Swerve.Controls.kAzimuthDeadband) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

  private final SwerveRequest.FieldCentricFacingAngle SpeakerCentricFacingAngleControl =
      new SwerveRequest.FieldCentricFacingAngle();
          /*.withDeadband(Swerve.Controls.kDriveDeadBand)
          .withRotationalDeadband(Swerve.Controls.kAzimuthDeadband)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagic);
           */

  private final SwerveRequest.RobotCentric RobotCentricControl =
      new SwerveRequest.RobotCentric()
          .withDeadband(Swerve.Controls.kDriveDeadBand)
          .withRotationalDeadband(Swerve.Controls.kAzimuthDeadband)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(Swerve.Controls.MaxSpeed);

  private final SendableChooser<Command> autoChooser;

  private void configureDriverBindings() {
    drivetrain.setDefaultCommand(
        new ConditionalCommand(
            new ConditionalCommand(
                drivetrain.applyRequest(
                    () -> {
                      return SpeakerCentricFacingAngleControl.withVelocityX(
                              -joystick.getLeftY() * Swerve.Controls.MaxSpeed) // Drive forward with
                          // negative Y (forward)
                          .withVelocityY(
                              -joystick.getLeftX()
                                  * Swerve.Controls.MaxSpeed) // Drive left with negative X (left)
                          .withTargetDirection(drivetrain.getHeading.get());
                    }),
                drivetrain.applyRequest(
                    () -> {
                      return GenericFieldCentricControl.withVelocityX(
                              -joystick.getLeftY() * Swerve.Controls.MaxSpeed) // Drive forward with
                          // negative Y (forward)
                          .withVelocityY(
                              -joystick.getLeftX()
                                  * Swerve.Controls.MaxSpeed) // Drive left with negative X (left)
                          .withRotationalRate(-joystick.getRightX() * Swerve.Controls.MaxSpeed);
                    }),
                () -> drivetrain.getDriveMode() == Swerve.Controls.DRIVE_MODE.SpeakerCentric),
            drivetrain.applyRequest(
                () -> {
                  drivetrain.setDriveMode(Swerve.Controls.DRIVE_MODE.GenericFieldCentric);
                  return GenericFieldCentricControl.withVelocityX(
                          -joystick.getLeftY() * Swerve.Controls.MaxSpeed) // Drive forward with
                      // negative Y (forward)
                      .withVelocityY(
                          -joystick.getLeftX()
                              * Swerve.Controls.MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(-joystick.getRightX() * Swerve.Controls.MaxSpeed);
                }),
            () -> Math.abs(-joystick.getRightX()) > 0.1));

    joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .circle()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .circle()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    joystick.touchpad().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    joystick.L1().onTrue(new RunCommand(drivetrain::toggleDriveMode, drivetrain));

    joystick.L2().onTrue(new InstantCommand(drivetrain::seedTemp));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    } else {
      drivetrain.registerTelemetry(logger::telemeterize);
    }
  }

  // xbox
  public void configureCoBindings() {

    Command c_command = new IntakeCommand(intake, indexer);
    cojoystick.leftBumper().whileTrue(c_command);
    cojoystick
        .leftTrigger()
        .onTrue(new InstantCommand(shooter::spinUp))
        .onFalse(new InstantCommand(shooter::stopShooter));
    cojoystick
        .rightTrigger()
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(indexer::Pass),
                new InstantCommand(indexer::stopIndexer),
                () -> shooter.isShooting()))
        .onFalse(new InstantCommand(indexer::stopIndexer));
    cojoystick
        .rightBumper()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(indexer::stopIndexer),
                new InstantCommand(shooter::stopShooter)));
    cojoystick.povUp().whileTrue(new InstantCommand(hanger::ExtendHanger));
    cojoystick.povDown().whileTrue(new InstantCommand(hanger::RetractHanger));
    cojoystick.povCenter().whileTrue(new InstantCommand(hanger::StopHanger));
    cojoystick
        .x()
        .onTrue(new InstantCommand(shooter::ampShot))
        .onFalse(new InstantCommand(shooter::stopShooter));
    cojoystick
        .b()
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(intake::outTake), new InstantCommand(indexer::reverseIndexer)))
        .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(indexer::stopIndexer), new InstantCommand(intake::stopIntake)));
  }

  public RobotContainer() {
    // Register Named Commands
    NamedCommands.registerCommand("IntakeOn", new IntakeCommand(intake, indexer));
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
                new InstantCommand(shooter::spinUp),
                new InstantCommand(indexer::Pass))
            .withTimeout(2.5));
    NamedCommands.registerCommand(
        "FunnyInake",
        new ParallelCommandGroup(
                new InstantCommand(intake::intake), new InstantCommand(indexer::Pass))
            .withTimeout(0.75));
    NamedCommands.registerCommand(
        "JustStop",
        new ParallelCommandGroup(
            new InstantCommand(indexer::stopIndexer), new InstantCommand(shooter::stopShooter)));

    // SpeakerCentricFacingAngleControl.HeadingController = new PhoenixPIDController(100, 0, 0);

    configureDriverBindings();
    configureCoBindings();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    autoChooser.addOption(
        "Choreo 4 Note auto test 1",
        new Autos.FourNoteCenterStart(drivetrain, intake, indexer, shooter));
    autoChooser.addOption(
        "Choreo i meter test", new Autos.OneMeterTest(drivetrain, intake, indexer, shooter));
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    // auto chooser on shuffleboard
    return autoChooser.getSelected();
    // return new Autos.FourNoteCenterStart(drivetrain, intake, indexer, shooter);
  }
}
