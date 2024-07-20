package com.team841.betaSwerve2024.Constants;

import com.team841.betaSwerve2024.Drive.Drivetrain;
import com.team841.betaSwerve2024.FeedBack;
import com.team841.betaSwerve2024.Superstructure.*;
import com.team841.betaSwerve2024.Vision.NoteDetection;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Manifest {
  public class SubsystemManifest {
    public static final Drivetrain drivetrain = Swerve.DriveTrain;
    public static final Intake intake = new Intake();

    public static final Shooter shooter = new Shooter();

    public static final Indexer indexer = new Indexer();

    public static final Arm arm = new Arm();

    public static final LED led = new LED();

    public static final Hanger hanger = new Hanger();

    public static final NoteDetection noteDetection = new NoteDetection();
  }

  public class JoystickManifest {
    public static final CommandXboxController joystick =
        new CommandXboxController(ConstantsIO.OI.driverPortLeft); // My joystick
    //public static final CommandXboxController cojoystick = new CommandXboxController(ConstantsIO.OI.codriverPort);

    public static final FeedBack FEED_BACK = new FeedBack();
  }
}
