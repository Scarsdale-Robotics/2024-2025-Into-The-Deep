package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name="Localization Test")
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        LocalizationSubsystem localization = new LocalizationSubsystem(
                new Pose2d(),
                robot.leftOdometer,
                robot.rightOdometer,
                robot.centerOdometer,
                robot.imu,
                telemetry
        );
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this,
                telemetry
        );

        waitForStart();

        double drive_speed = 0.5;
        while (opModeIsActive()) {

            // Manual control
            double x = 0, y = 0, turn = 0;
            if (Math.abs(gamepad1.left_stick_x) > 0.05)
                x = gamepad1.left_stick_x;
            if (Math.abs(gamepad1.left_stick_y) > 0.05)
                y = -gamepad1.left_stick_y;
            if (Math.abs(gamepad1.right_stick_x) > 0.05)
                turn = gamepad1.right_stick_x;
            drive.driveFieldCentric(drive_speed * x, drive_speed * y, drive_speed * turn);

            // Localize
            localization.update();

            // Draw robot on dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), localization.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }

    }

}
