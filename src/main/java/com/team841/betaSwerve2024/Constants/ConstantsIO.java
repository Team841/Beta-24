package com.team841.betaSwerve2024.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class ConstantsIO {

  public static Supplier<Boolean> isRedAlliance =
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      };

  public static boolean rumbleNeedsPing = false;

  public static class OI {
    public static final int driverPortLeft = 0; // controller USB port 0
    public static final int driverPortRight = 1; // controller USB port 1
    public static final int codriverPort = 2; // controller USB port 2

    public static final int SysIDCommandPort = 4; // for sysID
  }

  public static final class CANID {

    public static final int kIntake = 2;

    public static final int kIndexerTalon = 10;

    public static final int kHangerMotorLeft = 15;
    public static final int kHangerMotorRight = 3;

    public static final int kLeftArmJoint = 13;
    public static final int kRightArmJoint = 14;

    public static final int bottomShooter = 12;

    public static final int topShooter = 11;
  }
}
