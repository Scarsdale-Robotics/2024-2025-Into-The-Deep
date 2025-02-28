package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
@TeleOp(name = "ViirTeleop")

//don't merge changes, private branch.
public class viir_Practice extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack
        );
        waitForStart();
        while (opModeIsActive()) {
            double majestic = 102043204.5;
            double speed = 0.9;
            boolean breakLoopHehe = false;
            if (gamepad1.square && !breakLoopHehe) {
                breakLoopHehe = true;
            }
            if (breakLoopHehe) {
                telemetry.addData("sigma sigma boy sigma boy sigma boy", 9);
                telemetry.update();
            }
            if (!gamepad1.square) {
                breakLoopHehe = false;
            }
            double strafe = -gamepad1.left_stick_x * speed;
            double forward = gamepad1.left_stick_y * speed;
            double turn = gamepad1.right_stick_x * speed;
            drive.driveRobotCentric(strafe, forward, turn);
            double buffer = 0.2;
            boolean senseChangeActivated = false;
            if (gamepad1.circle && !senseChangeActivated) {
                senseChangeActivated = true;
            }
            if (senseChangeActivated) {
                strafe = -gamepad1.left_stick_x + buffer;
                turn = -gamepad1.right_stick_x - buffer;
                forward = gamepad1.left_stick_y - buffer;
            }
            if (senseChangeActivated && gamepad1.circle) {
                senseChangeActivated = false;
            }
            boolean speedUpToggled = false;
            if (gamepad1.a && !speedUpToggled) {
                speedUpToggled = true;
            }
            if (speedUpToggled) {
                speed = 1;
            }
            if (!gamepad1.a) {
                speedUpToggled = false;
            }
        }
    }
}