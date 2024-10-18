package org.firstinspires.ftc.teamcode.opmodes.calibration.ExampleSynchroPather.translation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;

@Autonomous(name="Example SynchroPather Circular Spline Auto")
public class ExampleSynchroPatherCircularSplineAuto extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, -12, new Rotation2d(Math.PI/2)), this);
        initSynchronizer();

        waitForStart();

        while (opModeIsActive()) {
            while (opModeIsActive() && !gamepad1.square) {
                robot.localization.update();
            }
            synchronizer.start();
            while (opModeIsActive() && synchronizer.update()) {
                robot.localization.update();
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), robot.localization.getPose());
                if (robot.opMode.gamepad1.triangle)
                    Drawing.drawTargetPose(packet.fieldOverlay(), new Pose2d(robot.drive.targetX, robot.drive.targetY, new Rotation2d(robot.drive.targetH)));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
            synchronizer.stop();
            robot.localization.update();
        }
    }


    private void initSynchronizer() {
        // Translation plan
        CRSplineTranslation spline1 = new CRSplineTranslation(0,
                new TranslationState(0, -12),
                new TranslationState(12, 0),
                new TranslationState(0, 12),
                new TranslationState(-12, 0),
                new TranslationState(0, -12)
        );
        CRSplineTranslation spline2 = new CRSplineTranslation(spline1.getEndTime(),
                new TranslationState(0, -12),
                new TranslationState(-12, 0),
                new TranslationState(0, 12),
                new TranslationState(12, 0),
                new TranslationState(0, -12)
        );
        TranslationPlan translationPlan = new TranslationPlan(robot,
                spline1,
                spline2
        );

        // Rotation plan
        LinearRotation rotation = new LinearRotation(new TimeSpan(0, spline2.getEndTime()),
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(0))
        );
        RotationPlan rotationPlan = new RotationPlan(robot,
                rotation
        );

        // Synchronizer
        this.synchronizer = new Synchronizer(
                translationPlan
//                ,rotationPlan
        );
    }

}
