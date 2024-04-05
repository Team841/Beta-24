package com.team841.betaSwerve2024.Drive;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.team841.betaSwerve2024.Constants.ConstantsIO;
import com.team841.betaSwerve2024.Constants.Field;
import com.team841.betaSwerve2024.Constants.Manifest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.sql.Struct;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ComputeThread {
    protected static final int START_THREAD_PRIORITY = 2;
    protected final Thread compute_thread;

    protected volatile boolean running = false;

    private volatile double ControllerOutput;
    private volatile boolean NeedControllerOutput;
    private final PhoenixPIDController HeadingController = new PhoenixPIDController(13, 0, 0);
    private double mostRecentTimeStamp;
    private Pose2d mostRecentRobotPose;

    public ComputeThread(){
        compute_thread = new Thread(this::compute);
        compute_thread.setDaemon(true);
    }

    /**
     * Starts the Compute thread.
     */
    public void start() {
        running = true;
        compute_thread.start();
    }

    /**
     * Stops the Compute thread.
     */
    public void stop() {
        stop(0);
    }

    public Supplier<Double> getHeading = ()->{return this.ControllerOutput;};

    public void isSpeakerCentric(boolean value){
        if (value){
            if(this.NeedControllerOutput){
                return;
            } else{
                this.HeadingController.reset();
                this.NeedControllerOutput = true;
                return;
            }
        } else{
            this.NeedControllerOutput = false;
        }
    }

    public void update(Pose2d pose, double time){
        this.mostRecentRobotPose = pose;
        this.mostRecentTimeStamp = time;
    }

    /**
     * Stops the Compute thread with a timeout.
     *
     * @param millis The time to wait in milliseconds
     */
    public void stop(long millis) {
        running = false;
        try {
            compute_thread.join(millis);
        } catch (final InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    public void compute(){
        //Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

        if (this.mostRecentRobotPose == null){
            return;
        }

        while (running){

            double angleToFace;



            if (ConstantsIO.isRedAlliance.get()) { // Red side
                angleToFace = Math.atan((Field.kRedSpeakerPose2d.getY() - this.mostRecentRobotPose.getY()) / (Field.kRedSpeakerPose2d.getX() - this.mostRecentRobotPose.getX()));
            } else { // blue side
                angleToFace =
                        Math.atan(
                                (Field.kBlueSpeakerPose2d.getY() - this.mostRecentRobotPose.getY())
                                        / (Field.kBlueSpeakerPose2d.getX() - this.mostRecentRobotPose.getX())) + 180;
            }

            this.ControllerOutput = HeadingController.calculate(
                    this.mostRecentRobotPose.getRotation().getRadians(), angleToFace, this.mostRecentTimeStamp);

            SmartDashboard.putNumber("OUTPUT", this.ControllerOutput);
        }
    }
}
