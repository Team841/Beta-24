package com.team841.betaSwerve2024.Drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.Swerve;
import com.team841.betaSwerve2024.Vision.LimelightHelpers;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class Drivetrain extends SwerveDrivetrain implements Subsystem {

  /*
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("LimelightTest");

  //final StructPublisher<Pose2d> lime = table.getStructTopic("Limelight Pose", ).publish();

  Topic genLime = inst.getTopic("LimelightTest");
  Topic poseTgen = inst.getTopic("LimelightTest");

  StructTopic limeT = inst.getStructTopic("LimeT", genLime.get)

  final StructPublisher<Pose2d> poseP;
  final StructPublisher<Pose2d> limeP;

   */

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable poseTest = inst.getTable("Pose Test");

  StructTopic<Pose2d> limeT = poseTest.getStructTopic("Limelight Pose", Pose2d.struct);
  StructTopic<Pose2d> ctreT = poseTest.getStructTopic("CTRE Pose", Pose2d.struct);
  StructTopic<Pose2d> shotT = poseTest.getStructTopic("SHOOOTER LIME LIGHT YEEEEEE", Pose2d.struct);

  StructPublisher<Pose2d> limeP = limeT.publish();
  StructPublisher<Pose2d> ctreP = ctreT.publish();
  StructPublisher<Pose2d> shotP = shotT.publish();

  public static Matrix<N3, N1> kVisionStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

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

    //this.setOperatorPerspectiveForward(ConstantsIO.isRedAlliance.get() ? new Rotation2d(Math.PI) : new Rotation2d(0.0));

    kVisionStdDevs.set(0,0, 2);
    kVisionStdDevs.set(1,0,2);
    kVisionStdDevs.set(2, 0, Math.PI*2);

    ConfigureMotors();
    configurePathplanner();
  }

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    //this.setOperatorPerspectiveForward(ConstantsIO.isRedAlliance.get() ? new Rotation2d(Math.PI) : new Rotation2d(0.0));

    kVisionStdDevs.set(0,0, 2);
    kVisionStdDevs.set(1,0,2);
    kVisionStdDevs.set(2, 0, Math.PI*2);

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

  public void seedTemp() {
    this.seedFieldRelative(
        GeometryUtil.flipFieldPose(
            new Pose2d(1.3865381479263306, 4.631037712097168, new Rotation2d(3.14159))));
  }

  @Override
  public void periodic() {
    if (LimelightHelpers.getTV(Swerve.Vision.kLimelightFrontName)){
      this.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue(Swerve.Vision.kLimelightFrontName), Timer.getFPGATimestamp(), kVisionStdDevs);
    }

    ctreP.set(this.getState().Pose);
    limeP.set(LimelightHelpers.getBotPose2d_wpiBlue("limelight-front"));
    shotP.set(LimelightHelpers.getBotPose2d_wpiBlue("limelight-back"));

    SmartDashboard.putString("vison matrix", kVisionStdDevs.toString());
  }
}
