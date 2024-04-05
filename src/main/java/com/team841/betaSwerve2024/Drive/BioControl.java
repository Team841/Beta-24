package com.team841.betaSwerve2024.Drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.Field;
import com.team841.betaSwerve2024.Constants.Manifest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class BioControl implements SwerveRequest {

  /**
   * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   */
  public double VelocityX = 0;

  /**
   * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   */
  public double VelocityY = 0;

  /**
   * The angular rate to rotate at, in radians per second. Angular rate is defined as
   * counterclockwise positive, so this determines how fast to turn counterclockwise.
   */
  public double RotationalRate = 0;

  public Rotation2d TargetDirection = new Rotation2d();

  /** Drive in Speaker Centric Mode */
  public boolean InSpeakerCentric = false;

  /** The allowable deadband of the request. */
  public double Deadband = 0;

  /** The rotational deadband of the request. */
  public double RotationalDeadband = 0;

  /**
   * The center of rotation the robot should rotate around. This is (0,0) by default, which will
   * rotate around the center of the robot.
   */
  public Translation2d CenterOfRotation = new Translation2d();

  /** The type of control request to use for the drive motor. */
  public SwerveModule.DriveRequestType DriveRequestType =
          SwerveModule.DriveRequestType.OpenLoopVoltage;

  /** The type of control request to use for the steer motor. */
  public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

  /** The perspective to use when determining which direction is forward. */
  public ForwardReference ForwardReference = SwerveRequest.ForwardReference.OperatorPerspective;

  /** The last applied state in case we don't have anything to drive. */
  protected SwerveModuleState[] m_lastAppliedState = null;


  public StatusCode apply(
          SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    double toApplyX = VelocityX;
    double toApplyY = VelocityY;
    if (ForwardReference == SwerveRequest.ForwardReference.OperatorPerspective) {
      /* If we're operator perspective, modify the X/Y translation by the angle */
      Translation2d tmp = new Translation2d(toApplyX, toApplyY);
      tmp = tmp.rotateBy(parameters.operatorForwardDirection);
      toApplyX = tmp.getX();
      toApplyY = tmp.getY();
    }
    double toApplyOmega = RotationalRate;

    if (InSpeakerCentric) {
      Manifest.SubsystemManifest.drivetrain.compute.isSpeakerCentric(true);
      toApplyOmega = Manifest.SubsystemManifest.drivetrain.compute.getHeading.get();
    } else{
      Manifest.SubsystemManifest.drivetrain.compute.isSpeakerCentric(false);
    }

    if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
      toApplyX = 0;
      toApplyY = 0;
    }
    if (Math.abs(toApplyOmega) < RotationalDeadband) {
      toApplyOmega = 0;
    }

    ChassisSpeeds speeds =
            ChassisSpeeds.discretize(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            toApplyX, toApplyY, toApplyOmega, parameters.currentPose.getRotation()),
                    parameters.updatePeriod);

    var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

    for (int i = 0; i < modulesToApply.length; ++i) {
      modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
    }

    return StatusCode.OK;
  }

  /**
   * Sets the velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   *
   * @param velocityX Velocity in the X direction, in m/s
   * @return this request
   */
  public BioControl withVelocityX(double velocityX) {
    this.VelocityX = velocityX;
    return this;
  }

  /**
   * Sets the velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   *
   * @param velocityY Velocity in the Y direction, in m/s
   * @return this request
   */
  public BioControl withVelocityY(double velocityY) {
    this.VelocityY = velocityY;
    return this;
  }

  /**
   * The angular rate to rotate at, in radians per second. Angular rate is defined as
   * counterclockwise positive, so this determines how fast to turn counterclockwise.
   *
   * @param rotationalRate Angular rate to rotate at, in radians per second
   * @return this request
   */
  public BioControl withRotationalRate(double rotationalRate) {
    this.RotationalRate = rotationalRate;
    return this;
  }

  /**
   * Sets the allowable deadband of the request.
   *
   * @param deadband Allowable deadband of the request
   * @return this request
   */
  public BioControl withDeadband(double deadband) {
    this.Deadband = deadband;
    return this;
  }

  /**
   * Sets the rotational deadband of the request.
   *
   * @param rotationalDeadband Rotational deadband of the request
   * @return this request
   */
  public BioControl withRotationalDeadband(double rotationalDeadband) {
    this.RotationalDeadband = rotationalDeadband;
    return this;
  }

  /**
   * Sets the center of rotation of the request
   *
   * @param centerOfRotation The center of rotation the robot should rotate around.
   * @return this request
   */
  public BioControl withCenterOfRotation(Translation2d centerOfRotation) {
    this.CenterOfRotation = centerOfRotation;
    return this;
  }

  /**
   * Sets the type of control request to use for the drive motor.
   *
   * @param driveRequestType The type of control request to use for the drive motor
   * @return this request
   */
  public BioControl withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
    this.DriveRequestType = driveRequestType;
    return this;
  }

  /**
   * Sets the type of control request to use for the steer motor.
   *
   * @param steerRequestType The type of control request to use for the steer motor
   * @return this request
   */
  public BioControl withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
    this.SteerRequestType = steerRequestType;
    return this;
  }

  public BioControl withSpeakerCentricMode(Boolean mode) {
    this.InSpeakerCentric = mode;
    return this;
  }

  public BioControl withTargetDirection(Rotation2d direction) {
    this.TargetDirection = direction;
    return this;
  }
}