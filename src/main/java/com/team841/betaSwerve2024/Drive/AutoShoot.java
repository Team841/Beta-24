package com.team841.betaSwerve2024.Drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.Field;
import com.team841.betaSwerve2024.Superstructure.Indexer;
import com.team841.betaSwerve2024.Superstructure.Shooter;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShoot extends Command {

  private ProfiledPIDController TurnController = new ProfiledPIDController(10.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0, 0), 0.2);

  private SwerveRequest.ApplyChassisSpeeds turnSpeeds = new SwerveRequest.ApplyChassisSpeeds();

  private Drivetrain f_Drivetrain;
  private Indexer f_indexer;
  private Shooter f_Shooter;

  // LimelightHelpers.PoseEstimate currentLimelightPose;

  Rotation2d aimGoal;

  double passStartTime = -1;

  public AutoShoot(Drivetrain omg_Drivetrain, Indexer omg_indexer, Shooter omg_Shooter) {
    this.f_Drivetrain = omg_Drivetrain;
    this.f_indexer = omg_indexer;
    this.f_Shooter = omg_Shooter;
    addRequirements(f_Drivetrain, f_indexer, f_Shooter);
  }

  @Override
  public void initialize() {
    // this.currentLimelightPose = f_Drivetrain.getLimeLightPoses();

    this.aimGoal = f_Drivetrain.getHeadingToSpeaker.get();
    this.f_Shooter.spinUp();
    this.TurnController.setIntegratorRange(-1.0, 1.0);
    this.TurnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    // this.getState().Pose.getRotation().minus(Field.kRedSpeakerPose2d.getRotation());
    if ((f_Drivetrain.getState().Pose.getY() - Field.kBlueSpeakerPose2d.getY()) < 0.5) {
      this.aimGoal = new Rotation2d(Math.PI);
    } else if (ConstantsIO.isRedAlliance.get()) {
      this.aimGoal =
          new Rotation2d(
              f_Drivetrain.getState().Pose.getX() - Field.kRedSpeakerPose2d.getX(),
              f_Drivetrain.getState().Pose.getY() - Field.kRedSpeakerPose2d.getY());
    } else {
      this.aimGoal =
          new Rotation2d(
              f_Drivetrain.getState().Pose.getX() - Field.kBlueSpeakerPose2d.getX(),
              f_Drivetrain.getState().Pose.getY() - Field.kBlueSpeakerPose2d.getY());
    }

    var rotationConstraints =
            new TrapezoidProfile.Constraints(
                    Math.toRadians(540), Math.toRadians(720));

    double rotationFeedback =
        TurnController.calculate(
            f_Drivetrain.getState().Pose.getRotation().getRadians(),
            new TrapezoidProfile.State(aimGoal.getRotations(), 0),
            rotationConstraints);

    double rotationFF = TurnController.getSetpoint().velocity;

    f_Drivetrain.applyRequest(
        () ->
            turnSpeeds.withSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0.0,
                    0.0,
                    rotationFeedback + rotationFF,
                    f_Drivetrain.getState().Pose.getRotation())));

    if (TurnController.atSetpoint() && f_Shooter.isHighShot()) {
      f_indexer.Pass();
      if (this.passStartTime == -1)
        this.passStartTime = Timer.getFPGATimestamp();
    }
  }

  @Override
  public void end(boolean interrupted) {
    f_Drivetrain.applyRequest(
        () ->
            new SwerveRequest.ApplyChassisSpeeds()
                .withSpeeds(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0.0, 0.0, 0.0, f_Drivetrain.getState().Pose.getRotation())));
  }

  @Override
  public boolean isFinished() {
    return this.passStartTime == -1 ? false : Timer.getFPGATimestamp() - passStartTime > 0.8;
  }
}
