package com.team841.betaSwerve2024.Autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team841.betaSwerve2024.Drive.Drivetrain;
import com.team841.betaSwerve2024.Superstructure.Indexer;
import com.team841.betaSwerve2024.Superstructure.Intake;
import com.team841.betaSwerve2024.Superstructure.IntakeCommand;
import com.team841.betaSwerve2024.Superstructure.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;

import java.util.List;

public class CoreAutonomousSequence extends SequentialCommandGroup {

  private static final double FIELD_LENGTH = 16.54;

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

  public Command FollowPath(PathPlannerPath trajectory) {
    return new FollowPathHolonomic(
        trajectory,
        a_drivetrain::getPose, // Robot pose supplier
        a_drivetrain
            ::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (ChassisSpeeds speeds) ->
            a_drivetrain.setControl(
                new SwerveRequest.ApplyChassisSpeeds()
                    .withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.28780635091142, // Drive base radius in meters. Distance from robot center to furthest
            // module. Calculated by sqrt(10.375^2*2)
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        a_drivetrain // Reference to this subsystem to set requirements
        );
  }

  public Pose2d GetStartingPose2d(String trajectory) {
    return PathPlannerPath.fromChoreoTrajectory(trajectory).getPreviewStartingHolonomicPose();
  }

  public void ResetStartingPoseFromTrajectory(String trajectory) {
    ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(trajectory);
    Pose2d startingPose = choreoTrajectory.getInitialPose();
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      a_drivetrain.seedFieldRelative(startingPose);
    } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      a_drivetrain.seedFieldRelative(GeometryUtil.flipFieldPose(startingPose));
    } else {
      this.end(true);
    }
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
