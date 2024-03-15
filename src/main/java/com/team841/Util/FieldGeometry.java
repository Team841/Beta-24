package com.team841.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldGeometry {
  private static final double FIELD_LENGTH = 16.54;

  /**
   * Flip a field position to the other side of the field, maintaining a blue alliance origin
   *
   * @param pos The position to flip
   * @return The flipped position
   */
  public static Translation2d flipFieldPosition(Translation2d pos) {
    return new Translation2d(FIELD_LENGTH - pos.getX(), pos.getY());
  }

  /**
   * Flip a field rotation to the other side of the field, maintaining a blue alliance origin
   *
   * @param rotation The rotation to flip
   * @return The flipped rotation
   */
  public static Rotation2d flipFieldRotation(Rotation2d rotation) {
    return new Rotation2d(Math.PI).minus(rotation);
  }

  /**
   * Flip a field pose to the other side of the field, maintaining a blue alliance origin
   *
   * @param pose The pose to flip
   * @return The flipped pose
   */
  public static Pose2d flipFieldPose(Pose2d pose) {
    return new Pose2d(
        flipFieldPosition(pose.getTranslation()), flipFieldRotation(pose.getRotation()));
  }
}
