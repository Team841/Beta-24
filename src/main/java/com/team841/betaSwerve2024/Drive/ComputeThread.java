package com.team841.betaSwerve2024.Drive;

import edu.wpi.first.wpilibj.Threads;

import java.sql.Struct;

public class ComputeThread {
    protected static final int START_THREAD_PRIORITY = 2;
    protected final Thread compute_thread;

    protected volatile boolean running = false;

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
        Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

        while (running){

        }
    }
}
