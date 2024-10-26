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
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.movements.LinearClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowState;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.movements.LinearElbow;

@Config
@Autonomous(name="Example SynchroPather Elbow Auto", group = "Calibration")
public class ExampleSynchroPatherElbowAuto extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer;

    public static double clawOpen = 1;
    public static double clawClosed = 0.91;

    public static double elbowUp = 0.15;
    public static double elbowDown = 0.38;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)), this);
        robot.inDep.setElbowPosition(elbowUp);
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
        // Claw plan
        LinearElbow elbow1 = new LinearElbow(0,
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );
        LinearElbow elbow2 = new LinearElbow(elbow1.getEndTime()+1,
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        ElbowPlan elbowPlan = new ElbowPlan(robot,
                elbow1,
                elbow2
        );

        // Synchronizer
        this.synchronizer = new Synchronizer(
                elbowPlan
        );
    }

}
