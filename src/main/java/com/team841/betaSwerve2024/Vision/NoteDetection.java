package com.team841.betaSwerve2024.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class NoteDetection implements Subsystem {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable poseTest = inst.getTable("Pose Test");

    StructTopic<Pose2d> limeT = poseTest.getStructTopic("Limelight Pose", Pose2d.struct);
    StructTopic<Pose2d> noteT = poseTest.getStructTopic("Note Pose", Pose2d.struct);

    StructPublisher<Pose2d> limeP = limeT.publish();
    StructPublisher<Pose2d> noteP = noteT.publish();

    // Constants
    private static final double CAMERA_HEIGHT = 0.51; // Height of the camera from the ground in meters
    private static final double TARGET_HEIGHT = 0.051; // Height of the target from the ground in meters

    public static Pose2d calculateTargetPose(double tx, double ty, double ta) {
        // Convert angles to radians
        double txRad = Math.toRadians(tx);
        double tyRad = Math.toRadians(ty);

        // Estimate the distance to the target using vertical angle and heights
        double distance = estimateDistanceToTarget(tyRad);

        // Calculate x, y, z coordinates
        double z = distance;
        double x = distance * Math.tan(txRad);
        double y = z * Math.tan(tyRad);

        // Calculate the rotation based on the horizontal angle
        Rotation2d rotation = new Rotation2d(txRad);

        // Create Translation2d
        Translation2d translation = new Translation2d(x, y);

        // Combine translation and rotation into Pose2d
        Pose2d pose = new Pose2d(translation, rotation);

        return pose;
    }

    private static double estimateDistanceToTarget(double tyRad) {
        // Use trigonometry to estimate the distance to the target
        double heightDifference = TARGET_HEIGHT - CAMERA_HEIGHT;
        return heightDifference / Math.tan(tyRad);
    }

    public NoteDetection(){
        limeP.set(new Pose2d(new Translation2d(10, 5), new Rotation2d()));
    }


    @Override
    public void periodic() {
        var tx = LimelightHelpers.getTX("Pipeline_Name");
        var ty = LimelightHelpers.getTY("Pipeline_Name");
        var ta = LimelightHelpers.getTA("Pipeline_Name");

        if (tx>0.05 && ty>0.05)
            noteP.set(calculateTargetPose(tx, ty, ta));

        System.out.println("Im Running");
    }
}
