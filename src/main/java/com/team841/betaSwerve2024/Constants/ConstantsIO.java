package com.team841.betaSwerve2024.Constants;

public class ConstantsIO {

  public static boolean rumbleNeedsPing = false;

  public static class OI {
    public static final int driverPortLeft = 0; // controller USB port 0
    public static final int driverPortRight = 1; // controller USB port 1
    public static final int codriverPort = 2; // controller USB port 2

    public static final int SysIDCommandPort = 4; // for sysID
  }

  public static final class CANID {

    public static final int kIntakeOne = 2;

    public static final class PWMPorts {
      // LED BLINKIN
      // public static final int kblinkin = 4;

      // Intake sensor
      public static final int Intake_Index_Sensor = 0;

      // Arm sensor
      // public static final int Arm_Index_Sensor = 1;

      // Indexer sensor
      // public static final int Kicker_Index_Sensor = 5;
    }
  }
}
