package com.team841.betaSwerve2024.Autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.team841.betaSwerve2024.Drive.Drivetrain;
import com.team841.betaSwerve2024.Superstructure.Indexer;
import com.team841.betaSwerve2024.Superstructure.Intake;
import com.team841.betaSwerve2024.Superstructure.Shooter;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Autos {
  public static class FourNoteCenterStart extends CoreAutonomousSequence {

    private final ChoreoTrajectory movePreloadShot = Choreo.getTrajectory("FourNoteAmpSide.1");
    private final ChoreoTrajectory moveLeftNoteShot = Choreo.getTrajectory("FourNoteAmpSide.2");
    private final ChoreoTrajectory moveMidNoteShot = Choreo.getTrajectory("FourNoteAmpSide.3");
    private final ChoreoTrajectory moveRightNoteShot = Choreo.getTrajectory("FourNoteAmpSide.4");

    public FourNoteCenterStart(Drivetrain drive, Intake intake, Indexer indexer, Shooter shooter) {
      super(drive, intake, indexer, shooter);

      addCommands(
          ShooterSpinUp(),
          FollowPath(movePreloadShot),
          JustShoot(),
          new WaitCommand(0.5),
          StopIndexer(),
          new ParallelCommandGroup(FollowPath(moveLeftNoteShot), Intake()),
          JustShoot(),
          new WaitCommand(0.5),
          StopIndexer(),
          new ParallelCommandGroup(FollowPath(moveRightNoteShot), Intake()),
          JustShoot(),
          new WaitCommand(0.5),
          SuperstructureStop());
    }
  }
}
