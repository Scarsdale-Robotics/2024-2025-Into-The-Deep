package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;

@TeleOp(name="Basic TeleOp")
public class BasicTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack
        );

        waitForStart();

        double speed = 0.5;
        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drive.driveRobotCentric(strafe, forward, turn);
        }

    }
}
