package org.firstinspires.ftc.teamcode.opmodes.calibration.ExampleSynchroPather;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
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

@Autonomous(name="Example SynchroPather Sample Detection Opmode", group = "Calibration")
public class ExampleSynchroPatherSampleDetection extends LinearOpMode {

    RobotSystem robot;
    Synchronizer scanForSamples;
    Synchronizer moveToSamples;

    volatile ArrayDeque<Double> loopTicks;
    volatile ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)), false, this);
        initTPS();
        waitForStart();

        // Scan for samples
        initScanForSamples();
        scanForSamples.start();
        while (opModeIsActive() && scanForSamples.update()) updateLoop();
        scanForSamples.stop();

        // Stop if no samples detected
        if (!robot.cv.detectedAnySamples()) {
            while (opModeIsActive()) {
                telemetry.addData("_", "NO SAMPLES DETECTED :( stopping opmode");
                telemetry.update();
            }
        }

        // Move to samples
        initMoveToSample();
        moveToSamples.start();
        while (opModeIsActive() && moveToSamples.update()) updateLoop();
        moveToSamples.stop();


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

        // Translation plan
        TranslationConstants.MAX_VELOCITY = 0.5*40d;
        TranslationConstants.MAX_ACCELERATION = 0.5*54d;

        LinearTranslation line = new LinearTranslation(0,
                currentState,
                targetState
        );

        LinearTranslation still = new LinearTranslation(new TimeSpan(line.getEndTime(), 10),
                targetState,
                targetState
        );

        TranslationPlan translationPlan = new TranslationPlan(robot,
                line,
                still
        );


        // Rotation plan
        RotationConstants.MAX_ANGULAR_VELOCITY = 3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 7.2;

        double targetTheta = targetState.minus(currentState).theta();
        LinearRotation rotation = new LinearRotation(0,
                new RotationState(currentPose),
                new RotationState(targetTheta)
        );
        RotationPlan rotationPlan = new RotationPlan(robot,
                rotation
        );

        this.moveToSamples = new Synchronizer(
                translationPlan,
                rotationPlan
        );
    }

}
