package com.team841.betaSwerve2024.Autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team841.betaSwerve2024.Drive.Drivetrain;
import com.team841.betaSwerve2024.Superstructure.Indexer;
import com.team841.betaSwerve2024.Superstructure.Intake;
import com.team841.betaSwerve2024.Superstructure.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.nio.file.Path;

public class Autos {
  public static class FourNoteCenterStart extends CoreAutonomousSequence {

    private final String movePreloadShotName = "FourNoteAmpSide.1";
    private final PathPlannerPath movePreloadShot =
        PathPlannerPath.fromChoreoTrajectory("FourNoteAmpSide.1");
    private final PathPlannerPath moveLeftNoteShot =
        PathPlannerPath.fromChoreoTrajectory("FourNoteAmpSide.2");
    private final PathPlannerPath moveMidNoteShot =
        PathPlannerPath.fromChoreoTrajectory("FourNoteAmpSide.3");
    private final PathPlannerPath moveRightNoteShot =
        PathPlannerPath.fromChoreoTrajectory("FourNoteAmpSide.4");

    public FourNoteCenterStart(Drivetrain drive, Intake intake, Indexer indexer, Shooter shooter) {
      super(drive, intake, indexer, shooter);

      addCommands(
          new InstantCommand(() -> ResetStartingPoseFromTrajectory(movePreloadShotName)),
          ShooterSpinUp(),
          FollowPath(movePreloadShot),
          JustShoot(),
          new WaitCommand(0.3),
          StopIndexer(),
          new ParallelRaceGroup(FollowPath(moveLeftNoteShot), Intake()),
          JustShoot(),
          new WaitCommand(0.3),
          StopIndexer(),
          new ParallelRaceGroup(FollowPath(moveMidNoteShot), Intake()),
          JustShoot(),
          new WaitCommand(0.3),
          StopIndexer(),
          new ParallelRaceGroup(FollowPath(moveRightNoteShot), Intake()),
          JustShoot(),
          new WaitCommand(0.3),
          SuperstructureStop());
    }
  }

  public static class OneMeterTest extends CoreAutonomousSequence {
    private final String startingPath = "i meter.1";
    private final PathPlannerPath oneMeterPath = PathPlannerPath.fromChoreoTrajectory(startingPath);

    public OneMeterTest(Drivetrain drive, Intake intake, Indexer indexer, Shooter shooter) {
      super(drive, intake, indexer, shooter);

      addCommands(
              new InstantCommand(() -> ResetStartingPoseFromTrajectory(startingPath)),
              FollowPath(oneMeterPath)
      );
    }
  }
}
