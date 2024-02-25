package com.team841.betaSwerve2024;

import com.team841.Util.BioCommandPS5Controller;
import com.team841.Util.BioCommandXboxController;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.Manifest;
import edu.wpi.first.wpilibj.GenericHID;

public class Rumble {
    private final BioCommandPS5Controller joystick =
            Manifest.JoystickManifest.joystick; // My joystick
    private final BioCommandXboxController cojoystick =
            Manifest.JoystickManifest.cojoystick;

    private int count = 0;

    public void Intaked(){
            ConstantsIO.rumbleNeedsPing = true;
            joystick.setRumble(GenericHID.RumbleType.kBothRumble, 1);
            cojoystick.setRumble(GenericHID.RumbleType.kBothRumble, 1);
            count += 1;
    }

    public void update() {
        if (count == 100)
        {
            ConstantsIO.rumbleNeedsPing = false;
            joystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            cojoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            count = 0;
        } else
            count += 1;
    }

    public boolean nowCounting(){
        return count != 0;
    }
}
