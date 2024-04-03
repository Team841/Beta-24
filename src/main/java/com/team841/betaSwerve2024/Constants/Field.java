package com.team841.betaSwerve2024.Constants;

import com.team841.betaSwerve2024.Util.FieldGeometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Field {
  public static Pose3d kBlueSpeakerPose3d = new Pose3d(0.0, 5.549, 2.002, new Rotation3d());
  public static Pose2d kBlueSpeakerPose2d =
      new Pose2d(kBlueSpeakerPose3d.getX(), kBlueSpeakerPose3d.getY(), new Rotation2d(0.0));

  public static Pose2d kRedSpeakerPose2d = FieldGeometry.flipFieldPose(kBlueSpeakerPose2d);

  public static Pose2d S_OSpeakerC =
      new Pose2d(1.3865381479263306, 4.631037712097168, new Rotation2d(3.14159));
}
