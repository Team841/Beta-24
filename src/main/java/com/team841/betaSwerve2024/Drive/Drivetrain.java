package com.team841.betaSwerve2024.Drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team841.Util.FieldGeometry;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.Field;
import com.team841.betaSwerve2024.Constants.Swerve;
import com.team841.betaSwerve2024.Constants.Swerve.Controls.DRIVE_MODE;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private DRIVE_MODE driveMode = DRIVE_MODE.GenericFieldCentric;

  private final SwerveRequest.FieldCentric GenericFieldCentricControl =
      new SwerveRequest.FieldCentric()
          .withDeadband(Swerve.Controls.kDriveDeadBand) // Add a 10% deadband
          .withRotationalDeadband(Swerve.Controls.kAzimuthDeadband) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  private final SwerveRequest.FieldCentricFacingAngle SpeakerCentricFacingAngleControl =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(Swerve.Controls.kDriveDeadBand)
          .withRotationalDeadband(Swerve.Controls.kAzimuthDeadband)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric RobotCentricControl =
      new SwerveRequest.RobotCentric()
          .withDeadband(Swerve.Controls.kDriveDeadBand)
          .withRotationalDeadband(Swerve.Controls.kAzimuthDeadband)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.ApplyChassisSpeeds autoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    this.SpeakerCentricFacingAngleControl.HeadingController = new PhoenixPIDController(10, 0, 0);

    ConfigureMotors();
    configurePathplanner();
  }

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    this.SpeakerCentricFacingAngleControl.HeadingController = new PhoenixPIDController(10, 0, 0);

    ConfigureMotors();
    configurePathplanner();
  }

  private void ConfigureMotors() {
    for (var CurrentModule : this.Modules) {
      CurrentModule.getDriveMotor()
          .getConfigurator()
          .apply(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(60)
                  .withSupplyCurrentLimitEnable(true));
      CurrentModule.getSteerMotor()
          .getConfigurator()
          .apply(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(60)
                  .withSupplyCurrentLimitEnable(true));
    }
  }

  public void configurePathplanner() {

    double driveBaseRadius = 0;

    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds) ->
            this.setControl(
                autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(
            new PIDConstants(10, 0, 0),
            new PIDConstants(10, 0, 0),
            Swerve.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, // Change this if the path needs to be flipped on red vs blue
        this); // Subsystem for requirements
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public DRIVE_MODE getDriveMode() {
    return this.driveMode;
  }

  public void setDriveMode(DRIVE_MODE mode) {
    this.driveMode = mode;
  }

  public void toggleDriveMode() {
    if (this.driveMode == DRIVE_MODE.GenericFieldCentric) {
      this.driveMode = DRIVE_MODE.SpeakerCentric;
    } else {
      this.driveMode = DRIVE_MODE.GenericFieldCentric;
    }
  }

  public Supplier<Rotation2d> getHeading =
      () -> {
        // this.getState().Pose.getRotation().minus(Field.kRedSpeakerPose2d.getRotation());
        double rot = Math.atan(
                  (this.getState().Pose.getY() - Field.kBlueSpeakerPose2d.getY())
                      / (this.getState().Pose.getX() - Field.kBlueSpeakerPose2d.getX()));
        if (Math.abs(this.getState().Pose.getY() - Field.kBlueSpeakerPose2d.getY()) < 0.5) {
          return new Rotation2d(Math.PI);
        } else if (ConstantsIO.isRedAlliance.get()) {
          return FieldGeometry.flipFieldRotation(new Rotation2d(rot));
        } else {
          return new Rotation2d(rot);
        }
      };

  public void JoystickDrive(double VelocityX, double VelocityY, Double RotationalRate) {
    if (!(Math.abs(RotationalRate) < 0.05)) {
      switch (this.driveMode) {
        case SpeakerCentric:
          this.setControl(
              SpeakerCentricFacingAngleControl.withVelocityX(
                      VelocityY * Swerve.Controls.MaxSpeed) // Drive forward with
                  // negative Y (forward)
                  .withVelocityY(
                      VelocityX * Swerve.Controls.MaxSpeed) // Drive left with negative X (left)
                  .withTargetDirection(
                      getHeading.get())); // Drive counterclockwise with negative X (left))
          break;
        case GenericFieldCentric:
          this.setControl(
              GenericFieldCentricControl.withVelocityX(
                      VelocityY * Swerve.Controls.MaxSpeed) // Drive forward with
                  // negative Y (forward)
                  .withVelocityY(
                      VelocityX * Swerve.Controls.MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(RotationalRate.doubleValue() * Swerve.Controls.MaxSpeed));
          break;
        default:
          break;
      }
    } else {
      this.setControl(
          GenericFieldCentricControl.withVelocityX(
                  VelocityY * Swerve.Controls.MaxSpeed) // Drive forward with
              // negative Y (forward)
              .withVelocityY(
                  VelocityX * Swerve.Controls.MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(
                  RotationalRate.doubleValue()
                      * Swerve.Controls.MaxSpeed)); // Drive counterclockwise with negative X (left)
    }
  }

  public void seedTemp() {
    this.seedFieldRelative(GeometryUtil.flipFieldPose(Field.S_OSpeakerC));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public Pose2d getPose() {
    return this.getState().Pose;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Pose", this.getState().Pose.toString());
    SmartDashboard.putString("DriveMODE", this.driveMode.toString());
    SmartDashboard.putNumber("heading", getHeading.get().getRadians());
  }
}
