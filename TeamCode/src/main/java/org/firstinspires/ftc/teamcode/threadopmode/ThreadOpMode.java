package org.firstinspires.ftc.teamcode.threadopmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

/**
 * A type of {@link LinearOpMode} that contains threads to be ran in parallel periodically.
 * Register threads with {@link ThreadOpMode#registerThread(OdometryThread)}
 */
public abstract class ThreadOpMode extends LinearOpMode {
    private List<OdometryThread> threads = new ArrayList<>();

    /**
     * Registers a new {@link OdometryThread} to be ran periodically.
     * Registered threads will automatically be started during {@link LinearOpMode#start()} and stopped during {@link LinearOpMode#stop()}.
     *
     * @param taskThread A {@link OdometryThread} object to be ran periodically.
     */
    public final void registerThread(OdometryThread taskThread) {
        threads.add(taskThread);
    }

    /**
     * Contains code to be ran before the OpMode is started. Similar to {@link LinearOpMode#init()}.
     */
    public abstract void mainInit();
    /**
     * Contains code to be ran periodically in the MAIN thread. Similar to {@link LinearOpMode#loop()}.
     */
    public abstract void mainLoop();

    /**
     * Should not be called by subclass.
     */
    @Override
    public void runOpMode() {
        mainInit();
        for(OdometryThread taskThread : threads) {
            taskThread.start();
        }
        waitForStart();
        while (opModeIsActive()) mainLoop();
        for(OdometryThread taskThread : threads) {
            taskThread.stop();
        }
    }

}