package org.firstinspires.ftc.teamcode.threadopmode;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

import java.util.ArrayDeque;

public class OdometryThread {

    Actions actions;
    TaskRunnable taskRunnable;

    volatile LinearOpMode opMode;
    volatile Telemetry telemetry;
    volatile LocalizationSubsystem localization;

    volatile ArrayDeque<Double> loopTicks;
    volatile ElapsedTime runtime;

    volatile RobotSystem robot;

    /** Creates an object that contains code to be ran periodically on a separate thread.
     * Can be registered to a {@link ThreadOpMode} with {@link ThreadOpMode#registerThread(OdometryThread)}.
     *
     */
    public OdometryThread(LinearOpMode opMode, Telemetry telemetry, LocalizationSubsystem localization, RobotSystem robot) {
        this.taskRunnable = new TaskRunnable();
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.localization = localization;
        this.robot = robot;
    }

    /**
     * An interface to be passed to a {@link OdometryThread} constructor.
     */
    public interface Actions {
        /**
         * Robot code to be ran periodically on its own thread.
         */
        public void loop();
    }

    public void start() {
        taskRunnable.start();
    }

    public void stop() {
        taskRunnable.stop();
    }

    class TaskRunnable implements Runnable {
        private Thread t;

        TaskRunnable() {

        }

        public void run() {
            boolean started = false;
            double targetTPS = 100;
            try {
                while(!t.isInterrupted() && (!started || opMode.opModeIsActive())) {
                    localization.update();
                    if (opMode.opModeIsActive()) started = true;

                    /////////////////
                    // TPS COUNTER //
                    /////////////////

                    double currentTime = runtime.seconds();
                    loopTicks.add(currentTime);
                    while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
                    robot.setTPS(loopTicks.size());

                    sleep((int)(1000d/targetTPS));

                }
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            stop();
        }

        public void start() {
            if (t == null) {
                t = new Thread(this);
                t.start();
            }

            loopTicks = new ArrayDeque<>();
            runtime = new ElapsedTime(0);
            runtime.reset();
        }

        public void stop() {
            t.interrupt();
        }
    }
}