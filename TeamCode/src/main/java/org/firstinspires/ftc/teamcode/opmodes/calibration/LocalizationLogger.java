package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name="Localization Logger", group="Calibration")
public class LocalizationLogger extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotSystem robot = new RobotSystem(hardwareMap, this);

        robot.localization.disableCamera();

        waitForStart();

        double speed = 0.3;
        while (opModeIsActive()) {

            double forward = -speed*gamepad1.left_stick_y;
            double strafe = speed*gamepad1.left_stick_x;
            double turn = speed*gamepad1.right_stick_x;

            robot.drive.driveRobotCentric(strafe, forward, turn);

            // Localize
            robot.localization.update();

            // Draw robot on dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.localization.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


        }

    }

}
