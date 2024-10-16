package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

@TeleOp(name="Basic TeleOp")
public class BasicTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack
        );
        LocalizationSubsystem localization = new LocalizationSubsystem(
                new Pose2d(0, 0, new Rotation2d(0)),
                robot.leftOdometer,
                robot.rightOdometer,
                robot.centerOdometer,
                telemetry
        );

        waitForStart();

        double speed = 1;
        while (opModeIsActive()) {
            double forward = -speed*gamepad1.left_stick_y;
            double strafe = speed*gamepad1.left_stick_x;
            double turn = speed*gamepad1.right_stick_x;

            drive.driveRobotCentric(strafe, forward, turn);

            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            telemetry.update();
        }

    }
}
