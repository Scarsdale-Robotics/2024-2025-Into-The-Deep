package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class BasicTeleopForTspmo extends LinearOpMode {
    @Override
    public void runOpMode () throws InterruptedException {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(
                robot.rightFront,
                robot.rightBack,
                robot.leftFront,
                robot.leftBack
        );
        waitForStart();
        while (opModeIsActive()) {
            double speed = 1;
            double strafe = -gamepad1.left_stick_x;
            double forward = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            drive.driveRobotCentric(strafe,forward, turn);
        }
    }
}
