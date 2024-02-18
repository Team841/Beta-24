package com.team841.betaSwerve2024.Constants;

import com.team841.betaSwerve2024.Drive.Drivetrain;
import com.team841.betaSwerve2024.Superstructure.Arm;
import com.team841.betaSwerve2024.Superstructure.Indexer;
import com.team841.betaSwerve2024.Superstructure.Intake;
import com.team841.betaSwerve2024.Superstructure.Shooter;

public class SubsystemManifest {
  public static final Drivetrain drivetrain = Swerve.DriveTrain;
  public static final Intake intake = new Intake();

  public static final Shooter shooter = new Shooter();

  public static final Indexer indexer = new Indexer();

  public static final Arm arm = new Arm();
}
