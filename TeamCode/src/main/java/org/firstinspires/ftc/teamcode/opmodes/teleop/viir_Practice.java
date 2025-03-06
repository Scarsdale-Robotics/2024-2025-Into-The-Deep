package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
@TeleOp(name = "ViirTeleop")
//hello just random stuff i do here to practice, i know a lot more than i can write trust

//don't merge changes, private branch.
public class viir_Practice extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))), false, this);

        //telemetry stufff (find out meaning of multipletelemetry)

        waitForStart();
        while (opModeIsActive()) {
            robot.localization.update();
            robot.logOdometry();
            double speed = 0.9;
            boolean displayTelemetry = false;
            double strafe = -gamepad1.left_stick_x * speed;
            double forward = gamepad1.left_stick_y * speed;
            double turn = gamepad1.right_stick_x * speed;
            double buffer = 0.2;
            if (gamepad1.square && !displayTelemetry) {
                displayTelemetry = true;
            }
            if (displayTelemetry) {
                telemetry.addData("Strafe: ", strafe);
                telemetry.addData("Turn: ", turn);
                telemetry.addData("Forward: ", forward);
                telemetry.update();
            }
            if (!gamepad1.square) {
                displayTelemetry = false;
            }
            robot.drive.driveFieldCentric(strafe , forward, turn, Math.toDegrees(robot.localization.getH()));
            boolean senseChangeActivated = false;
            if (gamepad1.circle && !senseChangeActivated) {
                senseChangeActivated = true;
            }
            if (senseChangeActivated) {
                strafe = -gamepad1.left_stick_x + buffer;
                turn = gamepad1.right_stick_x - buffer;
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