package com.team841.betaSwerve2024.Drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.team841.betaSwerve2024.Constants.Swerve;
import com.team841.betaSwerve2024.Superstructure.Indexer;
import com.team841.betaSwerve2024.Superstructure.Shooter;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoShoot extends Command {

  private ProfiledPIDController TurnController = Swerve.TurnController;

  private TrapezoidProfile.Constraints rotationConstraints = Swerve.rotationConstraints;

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
    // this.TurnController.setIntegratorRange(-1.0, 1.0);
    this.TurnController.enableContinuousInput(-Math.PI, Math.PI);
    this.TurnController.setTolerance(0.5, 0.5);
    this.TurnController.setIntegratorRange(-1, 1);
    this.TurnController.reset(f_Drivetrain.getState().Pose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    // this.getState().Pose.getRotation().minus(Field.kRedSpeakerPose2d.getRotation());

    double rotationFeedback =
        TurnController.calculate(
            f_Drivetrain.getState().Pose.getRotation().getRadians(),
            new TrapezoidProfile.State(aimGoal.getRadians(), 0),
            rotationConstraints);

    double rotationFF = TurnController.getSetpoint().velocity;

    SmartDashboard.putNumber("RotationFeedback", rotationFeedback);
    SmartDashboard.putNumber("RotationFF", rotationFF);

    f_Drivetrain.setControl(
        turnSpeeds.withSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0.0,
                0.0,
                rotationFeedback + rotationFF,
                f_Drivetrain.getState().Pose.getRotation())));

    // Math.abs(f_Drivetrain.getState().Pose.getRotation().getDegrees() - aimGoal.getDegrees()) < 10
    if (TurnController.atGoal() && f_Shooter.isHighShot()) {
      f_indexer.Pass();
      if (this.passStartTime == -1) this.passStartTime = Timer.getFPGATimestamp();
    }

    SmartDashboard.putBoolean("At SETPOINT", TurnController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    f_Shooter.stopShooter();
    f_indexer.stopIndexer();
    this.passStartTime = -1;
  }

  @Override
  public boolean isFinished() {
    return this.passStartTime != -1 && Timer.getFPGATimestamp() - passStartTime > 0.8;
  }
}
