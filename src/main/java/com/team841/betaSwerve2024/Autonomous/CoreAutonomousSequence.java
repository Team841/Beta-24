package com.team841.betaSwerve2024.Autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.team841.betaSwerve2024.Drive.Drivetrain;
import com.team841.betaSwerve2024.Superstructure.Indexer;
import com.team841.betaSwerve2024.Superstructure.Intake;
import com.team841.betaSwerve2024.Superstructure.IntakeCommand;
import com.team841.betaSwerve2024.Superstructure.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;

public class CoreAutonomousSequence extends SequentialCommandGroup {

  public Drivetrain a_drivetrain;
  public Intake a_intake;
  public Indexer a_indexer;
  public Shooter a_shooter;

  public CoreAutonomousSequence(Drivetrain drive, Intake intake, Indexer indexer, Shooter shooter) {
    this.a_drivetrain = drive;
    this.a_intake = intake;
    this.a_indexer = indexer;
    this.a_shooter = shooter;

    addRequirements(drive, intake, indexer, shooter);
  }

  public Command FollowPath(ChoreoTrajectory trajectory) {
    return Choreo.choreoSwerveCommand(
        trajectory,
        a_drivetrain::getPose,
        new PIDController(10, 0, 0),
        new PIDController(10, 0, 0),
        new PIDController(10, 0, 0),
        (ChassisSpeeds speeds) ->
            a_drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
        () ->
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
        ,
        a_drivetrain);
  }

  public Command Intake() {
    return new IntakeCommand(a_intake, a_indexer);
  }

  public Command Shoot() {
    return new SequentialCommandGroup(
            new InstantCommand(a_shooter::spinUp),
            new WaitCommand(1),
            new InstantCommand(a_indexer::Pass))
        .withTimeout(3);
  }

  public Command ShooterSpinUp() {
    return new InstantCommand(a_shooter::spinUp);
  }

  public Command JustShoot() {
    return new InstantCommand(a_indexer::Pass);
  }

  public Command AllSubystems() {
    return new ParallelCommandGroup(
        new InstantCommand(a_intake::intake),
        new InstantCommand(a_shooter::spinUp),
        new InstantCommand(a_indexer::Pass));
  }

  public Command SuperstructureStop() {
    return new ParallelCommandGroup(
        new InstantCommand(a_indexer::stopIndexer), new InstantCommand(a_shooter::stopShooter));
  }

  public Command StopIndexer() {
    return new InstantCommand(a_indexer::stopIndexer);
  }
}
