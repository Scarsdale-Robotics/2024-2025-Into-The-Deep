package org.firstinspires.ftc.teamcode.opmodes.calibration.ExampleSynchroPather;

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
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

@Autonomous(name="Example SynchroPather Star Auto")
public class ExampleSynchroPatherStarAuto extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)), this);
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
                new TranslationState(0, 0),
                new TranslationState(6, 3),
                new TranslationState(6, 9),
                new TranslationState(12, 12),
                new TranslationState(18, 9),
                new TranslationState(18, 3),
                new TranslationState(24, 0),
                new TranslationState(18, -3),
                new TranslationState(18, -9),
                new TranslationState(12, -12),
                new TranslationState(6, -9),
                new TranslationState(6, -3),
                new TranslationState(0, 0),
                new TranslationState(-6, 3),
                new TranslationState(-6, 9),
                new TranslationState(-12, 12),
                new TranslationState(-18, 9),
                new TranslationState(-18, 3),
                new TranslationState(-24, 0),
                new TranslationState(-18, -3),
                new TranslationState(-18, -9),
                new TranslationState(-12, -12),
                new TranslationState(-6, -9),
                new TranslationState(-6, -3),
                new TranslationState(0, 0)
        );
        CRSplineTranslation spline2 = new CRSplineTranslation(spline1.getEndTime(),
                new TranslationState(0, 0),
                new TranslationState(3, 6),
                new TranslationState(9, 6),
                new TranslationState(12, 12),
                new TranslationState(9, 18),
                new TranslationState(3, 18),
                new TranslationState(0, 24),
                new TranslationState(-3, 18),
                new TranslationState(-9, 18),
                new TranslationState(-12, 12),
                new TranslationState(-9, 6),
                new TranslationState(-3, 6),
                new TranslationState(0, 0),
                new TranslationState(3, -6),
                new TranslationState(9, -6),
                new TranslationState(12, -12),
                new TranslationState(9, -18),
                new TranslationState(3, -18),
                new TranslationState(0, -24),
                new TranslationState(-3, -18),
                new TranslationState(-9, -18),
                new TranslationState(-12, -12),
                new TranslationState(-9, -6),
                new TranslationState(-3, -6),
                new TranslationState(0, 0)
        );
        TranslationPlan translationPlan = new TranslationPlan(robot,
                spline1,
                spline2
        );

        // Rotation plan
        LinearRotation rot1 = new LinearRotation(new TimeSpan(0, spline1.getEndTime()/2),
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(360))
        );
        LinearRotation rot2 = new LinearRotation(new TimeSpan(spline1.getEndTime()/2, spline1.getEndTime()),
                new RotationState(Math.toRadians(360)),
                new RotationState(Math.toRadians(0))
        );
        LinearRotation rot3 = new LinearRotation(new TimeSpan(spline1.getEndTime(), 3*spline1.getEndTime()/2),
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(360))
        );
        LinearRotation rot4 = new LinearRotation(new TimeSpan(3*spline1.getEndTime()/2, spline2.getEndTime()),
                new RotationState(Math.toRadians(360)),
                new RotationState(Math.toRadians(0))
        );
        RotationPlan rotationPlan = new RotationPlan(robot,
                rot1,
                rot2,
                rot3,
                rot4
        );


        // Lift plan
        LinearLift lift1 = new LinearLift(new TimeSpan(0, spline1.getEndTime()),
                new LiftState(0),
                new LiftState(4000)
        );
        LinearLift lift2 = new LinearLift(new TimeSpan(spline1.getEndTime(), spline2.getEndTime()),
                new LiftState(4000),
                new LiftState(0)
        );

        LiftPlan liftPlan = new LiftPlan(robot,
                lift1,
                lift2
        );

        // Synchronizer
        this.synchronizer = new Synchronizer(
                translationPlan
                ,rotationPlan
                ,liftPlan
        );
    }

}
