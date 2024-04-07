package com.team841.betaSwerve2024;

import com.ctre.phoenix6.SignalLogger;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.Manifest;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final DigitalInput intakeSensor = new DigitalInput(0);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // SignalLogger.setPath("/media/sda1/");
    SignalLogger.enableAutoLogging(false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (!intakeSensor.get() && !(Manifest.JoystickManifest.FEED_BACK.nowCounting()))
      Manifest.JoystickManifest.FEED_BACK.Intaked();
    if (ConstantsIO.rumbleNeedsPing) Manifest.JoystickManifest.FEED_BACK.update();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //  SignalLogger.start();

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    // SignalLogger.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
