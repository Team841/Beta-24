package com.team841.betaSwerve2024;

import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.Manifest;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class FeedBack {
  private final CommandXboxController cojoystick =
      Manifest.JoystickManifest.cojoystick; // My joystick
  // private final CommandXboxController cojoystick = Manifest.JoystickManifest.cojoystick;

  private int count = 0;

  public void Intaked() {
    ConstantsIO.rumbleNeedsPing = true;
    // joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
    cojoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
    count += 1;
  }

  public void update() {
    if (count == 100) {
      ConstantsIO.rumbleNeedsPing = false;
      // joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      cojoystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      count = 0;
    } else count += 1;
  }

  public boolean nowCounting() {
    return count != 0;
  }
}
