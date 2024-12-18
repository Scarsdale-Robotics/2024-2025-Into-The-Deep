package org.firstinspires.ftc.teamcode.opmodes.calibration.ExampleSynchroPather;

import static org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightConstants.RESOLUTION_N;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.movements.LinearClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowState;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.movements.LinearElbow;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPipeline;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements.DisableLimelight;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements.EnableLimelight;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

import java.util.ArrayDeque;
import java.util.ArrayList;

@Autonomous(name="Example SynchroPather Sample Detection Opmode", group = "Calibration")
@Config
public class ExampleSynchroPatherSampleDetection extends LinearOpMode {

    RobotSystem robot;
    Synchronizer scanForSamples;
    Synchronizer moveToSamples;

    volatile ArrayDeque<Double> loopTicks;
    volatile ElapsedTime runtime;

    public static double armReachX = 19.5;
    private TranslationState armReach = new TranslationState(armReachX, 0);

    public static double clawOpen = ClawConstants.OPEN_POSITION;
    public static double clawClosed = ClawConstants.CLOSED_POSITION;

    public static double elbowUp = ElbowConstants.UP_POSITION;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;


    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)), false, this);
        initTPS();
        waitForStart();

        while (opModeIsActive()) {

            // Clear detections
            robot.cv.sample_probability_distribution = new double[RESOLUTION_N+1][RESOLUTION_N+1];
            robot.cv.detectedSamples = new ArrayList<>();

            while (!gamepad1.triangle && opModeIsActive());
            while (gamepad1.triangle && opModeIsActive());

            // Scan for samples
            initScanForSamples();
            scanForSamples.start();
            while (opModeIsActive() && scanForSamples.update()) updateLoop();
            scanForSamples.stop();

            // Stop if no samples detected
            if (!robot.cv.detectedAnySamples()) {
                telemetry.addData("_", "NO SAMPLES DETECTED :( stopping opmode");
                telemetry.update();
                continue;
            }

            // Move to samples
            initMoveToSample();
            moveToSamples.start();
            while (opModeIsActive() && moveToSamples.update()) updateLoop();
            moveToSamples.stop();

        }



    }

    private void initTPS() {
        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        robot.telemetry.addData("[MAIN] TPS", 0);
        robot.telemetry.update();
    }

    private void updateLoop() {
        /////////////////
        // TPS COUNTER //
        /////////////////

        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        robot.telemetry.addData("[MAIN] TPS", loopTicks.size());
        robot.telemetry.update();

        // Localization
        robot.localization.update();
        robot.logOdometry();
    }


    private void initScanForSamples() {
        EnableLimelight enable = new EnableLimelight(new TimeSpan(0,2), LimelightPipeline.SAMPLE_DETECTOR);
        DisableLimelight disable = new DisableLimelight(new TimeSpan(2,2.1));
        LimelightPlan limelightPlan = new LimelightPlan(robot, LimelightPlan.Color.YELLOW,
                enable,
                disable
        );

        this.scanForSamples = new Synchronizer(
                limelightPlan
        );
    }

    private void initMoveToSample() {
        Pose2d currentPose = robot.localization.getPose();
        TranslationState currentState = new TranslationState(currentPose);
        TranslationState targetState = new TranslationState(robot.cv.getClosestDetectedSample(currentPose));
        double targetTheta = targetState.minus(currentState).theta();
        targetState = targetState.minus(armReach.rotateBy(targetTheta));

        Pose2d closestSample = robot.cv.getClosestDetectedSample(currentPose);

        robot.telemetry.addData("closestSample.getX()", closestSample.getX());
        robot.telemetry.addData("closestSample.getY()", closestSample.getY());
        robot.telemetry.update();

        // Translation plan
        TranslationConstants.MAX_VELOCITY = 0.5*40d;
        TranslationConstants.MAX_ACCELERATION = 0.5*54d;
        RotationConstants.MAX_ANGULAR_VELOCITY = 3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 7.2;

        // Approach sample

        LinearTranslation linePickup = new LinearTranslation(0,
                currentState,
                targetState
        );

        LinearRotation rotationPickup = new LinearRotation(0,
                new RotationState(currentPose),
                new RotationState(targetTheta)
        );

        // Pick up sample
        LinearElbow elbowDownPickup = new LinearElbow(Math.max(linePickup.getEndTime(), rotationPickup.getEndTime()),
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawClosePickup = new LinearClaw(elbowDownPickup.getEndTime(),
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearElbow elbowUpPickup = new LinearElbow(clawClosePickup.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        // Approach drop off
        LinearTranslation lineDropoff = new LinearTranslation(elbowUpPickup.getEndTime(),
                targetState,
                new TranslationState(0,0)
        );

        LinearRotation rotationDropoff = new LinearRotation(lineDropoff.getStartTime(),
                new RotationState(targetTheta),
                new RotationState(Math.toRadians(90))
        );

        // Drop off sample
        LinearElbow elbowDownDropoff = new LinearElbow(Math.max(lineDropoff.getEndTime(), rotationDropoff.getEndTime()),
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawOpenDropoff = new LinearClaw(elbowDownDropoff.getEndTime(),
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        LinearElbow elbowUpDropoff = new LinearElbow(clawOpenDropoff.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        // Rotate back to starting position
        LinearRotation rotationReturnToStart = new LinearRotation(elbowUpDropoff.getEndTime(),
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(0))
        );




        TranslationPlan translationPlan = new TranslationPlan(robot,
                linePickup,
                lineDropoff
        );

        RotationPlan rotationPlan = new RotationPlan(robot,
                rotationPickup,
                rotationDropoff,
                rotationReturnToStart
        );

        ClawPlan clawPlan = new ClawPlan(robot,
                clawClosePickup,
                clawOpenDropoff
        );

        ElbowPlan elbowPlan = new ElbowPlan(robot,
                elbowDownPickup,
                elbowUpPickup,
                elbowDownDropoff,
                elbowUpDropoff
        );

        this.moveToSamples = new Synchronizer(
                translationPlan,
                rotationPlan,
                clawPlan,
                elbowPlan
        );
    }

}
