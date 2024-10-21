package org.firstinspires.ftc.teamcode.opmodes.calibration.ExampleSynchroPather.rotation;

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
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

@Autonomous(name="Example SynchroPather Full Rotation Auto", group = "Calibration")
public class ExampleSynchroPatherFullRotation extends LinearOpMode {

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

        // Rotation plan
        LinearRotation rotCCW = new LinearRotation(0,
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(360))
        );
        LinearRotation rotCW = new LinearRotation(rotCCW.getEndTime(),
                new RotationState(Math.toRadians(360)),
                new RotationState(Math.toRadians(0))
        );
        RotationPlan rotationPlan = new RotationPlan(robot,
                rotCCW,
                rotCW
        );

        // Translation plan
        LinearTranslation still = new LinearTranslation(new TimeSpan(0, rotCW.getEndTime()),
                new TranslationState(0, 0),
                new TranslationState(0, 0)
        );
        TranslationPlan translationPlan = new TranslationPlan(robot,
                still
        );

        // Synchronizer
        this.synchronizer = new Synchronizer(
                rotationPlan
                ,translationPlan
        );
    }

}
