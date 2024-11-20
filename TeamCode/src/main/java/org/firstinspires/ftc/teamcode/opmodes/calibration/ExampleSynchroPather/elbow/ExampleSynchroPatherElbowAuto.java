package org.firstinspires.ftc.teamcode.opmodes.calibration.ExampleSynchroPather.elbow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowState;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.movements.LinearElbow;

@Config
@Autonomous(name="Example SynchroPather Elbow Auto", group = "Calibration")
public class ExampleSynchroPatherElbowAuto extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer1, synchronizer2;

    public static double elbowUp = 0.275;
    public static double elbowDown = 0.53;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)), false, this);
        robot.inDep.setElbowPosition(elbowUp);
        initSynchronizers();

        waitForStart();

        while (opModeIsActive()) {
            // Sync 1
            while (opModeIsActive() && !gamepad1.square) {
                robot.localization.update();
            }
            synchronizer1.start();
            while (opModeIsActive() && synchronizer1.update()) {
                robot.localization.update();
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), robot.localization.getPose());
                if (robot.opMode.gamepad1.triangle)
                    Drawing.drawTargetPose(packet.fieldOverlay(), new Pose2d(robot.drive.targetX, robot.drive.targetY, new Rotation2d(robot.drive.targetH)));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
            synchronizer1.stop();

            // Sync 2
            while (opModeIsActive() && !gamepad1.square) {
                robot.localization.update();
            }
            synchronizer2.start();
            while (opModeIsActive() && synchronizer2.update()) {
                robot.localization.update();
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), robot.localization.getPose());
                if (robot.opMode.gamepad1.triangle)
                    Drawing.drawTargetPose(packet.fieldOverlay(), new Pose2d(robot.drive.targetX, robot.drive.targetY, new Rotation2d(robot.drive.targetH)));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
            synchronizer2.stop();
            robot.localization.update();
        }
    }


    private void initSynchronizers() {
        // Elbow plan
        LinearElbow elbow1_1 = new LinearElbow(0,
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );
        LinearElbow elbow2_1 = new LinearElbow(elbow1_1.getEndTime()+1,
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        ElbowPlan elbowPlan_1 = new ElbowPlan(robot,
                elbow1_1,
                elbow2_1
        );

        // Synchronizer 1
        this.synchronizer1 = new Synchronizer(
                elbowPlan_1
        );


        // Elbow plan
        LinearElbow elbow1_2 = new LinearElbow(0,
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );
        LinearElbow elbow2_2 = new LinearElbow(elbow1_1.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        ElbowPlan elbowPlan_2 = new ElbowPlan(robot,
                elbow1_2,
                elbow2_2
        );

        // Synchronizer 2
        this.synchronizer2 = new Synchronizer(
                elbowPlan_2
        );
    }

}
