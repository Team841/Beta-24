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
import com.team841.betaSwerve2024.Constants.Field;
import com.team841.betaSwerve2024.Constants.Swerve;
import com.team841.betaSwerve2024.Vision.LimelightHelpers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class Drivetrain extends SwerveDrivetrain implements Subsystem {

  /*
   * NetworkTableInstance inst = NetworkTableInstance.getDefault();
   * NetworkTable table = inst.getTable("LimelightTest");
   * 
   * //final StructPublisher<Pose2d> lime = table.getStructTopic("Limelight Pose",
   * ).publish();
   * 
   * Topic genLime = inst.getTopic("LimelightTest");
   * Topic poseTgen = inst.getTopic("LimelightTest");
   * 
   * StructTopic limeT = inst.getStructTopic("LimeT", genLime.get)
   * 
   * final StructPublisher<Pose2d> poseP;
   * final StructPublisher<Pose2d> limeP;
   * 
   */

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable poseTest = inst.getTable("Pose Test");

  StructTopic<Pose2d> limeT = poseTest.getStructTopic("Limelight Pose", Pose2d.struct);
  StructTopic<Pose2d> ctreT = poseTest.getStructTopic("CTRE Pose", Pose2d.struct);
  StructTopic<Pose2d> shotT = poseTest.getStructTopic("SHOOOTER LIME LIGHT YEEEEEE", Pose2d.struct);

  StructPublisher<Pose2d> limeP = limeT.publish();
  StructPublisher<Pose2d> ctreP = ctreT.publish();
  StructPublisher<Pose2d> shotP = shotT.publish();

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    this.setOperatorPerspectiveForward(
        ConstantsIO.isRedAlliance.get() ? new Rotation2d(Math.PI) : new Rotation2d(0.0));

    ConfigureMotors();
    configurePathplanner();
  }

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    this.setOperatorPerspectiveForward(
        ConstantsIO.isRedAlliance.get() ? new Rotation2d(Math.PI) : new Rotation2d(0.0));

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
        (speeds) -> this.setControl(
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
    m_simNotifier = new Notifier(
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

  public LimelightHelpers.PoseEstimate getLimeLightPoses() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(Swerve.Vision.kLimelightFrontName);
  }

  public Supplier<Rotation2d> getHeadingToSpeaker = () -> {
    Rotation2d aimGoal;

    if (ConstantsIO.isRedAlliance.get()) { // Red side
      if (Math.abs(this.getState().Pose.getY() - Field.kRedSpeakerPose2d.getY()) < 0.15) {
        // aimGoal = new Rotation2d(Math.toRadians(-1 *
        // this.getState().Pose.getRotation().getDegrees()));
        aimGoal = new Rotation2d(0);
      } else {
        aimGoal = new Rotation2d(
            Math.atan(
                (Field.kRedSpeakerPose2d.getY() - this.getState().Pose.getY())
                    / (Field.kRedSpeakerPose2d.getX() - this.getState().Pose.getX())));
      }
    } else { // blue side
      if (Math.abs(this.getState().Pose.getY() - Field.kBlueSpeakerPose2d.getY()) < 0.15) {
        // aimGoal = new
        // Rotation2d(Math.toRadians(this.getState().Pose.getRotation().getDegrees() -
        // 180));
        aimGoal = new Rotation2d(Math.PI);
      } else {
        aimGoal = new Rotation2d(
            Math.atan(
                (Field.kBlueSpeakerPose2d.getY() - this.getState().Pose.getY())
                    / (Field.kBlueSpeakerPose2d.getX() - this.getState().Pose.getX()))
                + 180);
      }
    }

    return aimGoal;
  };

  @Override
  public void periodic() {
   boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation("limelight", this.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(Math.abs(this.getState().speeds.omegaRadiansPerSecond) > 4 * Math.PI || mt2.tagCount <2) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
        if(!doRejectUpdate)
      {
        this.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,Math.PI*1.5));
        this.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
  }
    ctreP.set(this.getState().Pose);
    limeP.set(mt2.pose);

    SmartDashboard.putBoolean("2 tags", mt2.tagCount >= 2);

    SmartDashboard.putNumber("Turn angle", getHeadingToSpeaker.get().getDegrees());
    SmartDashboard.putNumber("Facing", this.getState().Pose.getRotation().getDegrees());

    
}
}
