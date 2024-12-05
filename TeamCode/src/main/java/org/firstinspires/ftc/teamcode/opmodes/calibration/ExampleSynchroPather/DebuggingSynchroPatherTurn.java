package org.firstinspires.ftc.teamcode.opmodes.calibration.ExampleSynchroPather;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

import java.util.ArrayDeque;

@Autonomous(name="Debugging SynchroPather Turn OpMode", group = "Calibration")
public class DebuggingSynchroPatherTurn extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer;
    TranslationPlan translationPlan;
    RotationPlan rotationPlan;

    volatile ArrayDeque<Double> loopTicks;
    volatile ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        initSynchronizer();
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)), false, this);
        this.translationPlan.setRobot(robot);
        this.rotationPlan.setRobot(robot);
        this.synchronizer = new Synchronizer(
//                translationPlan
                rotationPlan
        );


        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        robot.telemetry.addData("[MAIN] TPS", 0);
        robot.telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            while (opModeIsActive() && !gamepad1.square) {
                updateTPS();
                robot.localization.update();
                robot.logOdometry();
            }
            synchronizer.start();
            while (opModeIsActive() && synchronizer.update()) {
                updateTPS();
                robot.localization.update();
                robot.logOdometry();
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), robot.localization.getPose());
                if (robot.opMode.gamepad1.triangle)
                    Drawing.drawTargetPose(packet.fieldOverlay(), new Pose2d(robot.drive.targetX, robot.drive.targetY, new Rotation2d(robot.drive.targetH)));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
            synchronizer.stop();
            updateTPS();
            robot.localization.update();
            robot.logOdometry();
        }
    }

    private void updateTPS() {
        /////////////////
        // TPS COUNTER //
        /////////////////

        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        robot.telemetry.addData("[MAIN] TPS", loopTicks.size());
        robot.telemetry.update();
    }


    private void initSynchronizer() {
        TranslationConstants.MAX_VELOCITY = 0.5*40d;
        TranslationConstants.MAX_ACCELERATION = 54d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 7.2;

        LiftConstants.MAX_VELOCITY = 2200;
        LiftConstants.MAX_ACCELERATION = 2200;

        ClawConstants.MAX_VELOCITY = 15.111111111;
        ClawConstants.MAX_ACCELERATION = 30.22222;

        ElbowConstants.MAX_VELOCITY = 1.021739;
        ElbowConstants.MAX_ACCELERATION = 1.021739;
        // Translation plan
        LinearTranslation line1 = new LinearTranslation(0,
                new TranslationState(0, 0),
                new TranslationState(24, 0)
        );
        LinearTranslation line2 = new LinearTranslation(line1.getEndTime(),
                new TranslationState(24, 0),
                new TranslationState(0, 0)
        );
        translationPlan = new TranslationPlan(robot,
                line1,
                line2
        );

        // Rotation plan
        LinearRotation rotation1 = new LinearRotation(0,
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(180))
        );
        LinearRotation rotation2 = new LinearRotation(rotation1.getEndTime(),
                new RotationState(Math.toRadians(180)),
                new RotationState(Math.toRadians(0))
        );
        LinearRotation rotation3 = new LinearRotation(rotation2.getEndTime(),
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(-180))
        );
        LinearRotation rotation4 = new LinearRotation(rotation3.getEndTime(),
                new RotationState(Math.toRadians(-180)),
                new RotationState(Math.toRadians(0))
        );
        rotationPlan = new RotationPlan(robot,
                rotation1,
                rotation2,
                rotation3,
                rotation4
        );
    }

}
