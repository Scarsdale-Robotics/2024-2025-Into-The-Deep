package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;

@Config
@TeleOp(name="Basic TeleOp")
public class BasicTeleop extends LinearOpMode {

    public static double clawOpen = 1;
    public static double clawClosed = 0.91;

    public static double elbowUp = 0.15;
    public static double elbowDown = 0.38;

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
                new Pose2d(0, 0, new Rotation2d(Math.PI/2)),
                robot.leftOdometer,
                robot.rightOdometer,
                robot.centerOdometer,
                telemetry
        );

        robot.claw.setPosition(clawOpen);
        robot.elbow.setPosition(elbowUp);

        waitForStart();

        double speed = 1;
        boolean claw = false, toggleClaw = false; // false = claw open
        boolean elbow = true, toggleElbow = false; // true = elbow up
        while (opModeIsActive()) {
            localization.update();

            double forward = -speed*gamepad1.left_stick_y;
            double strafe = speed*gamepad1.left_stick_x;
            double turn = speed*gamepad1.right_stick_x;

            drive.driveFieldCentric(strafe, forward, turn, localization.getH());

            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            telemetry.update();


            ////////////////////
            // INDEP CONTROLS //
            ////////////////////

            // triangle = claw
            if (gamepad1.triangle && !toggleClaw) {
                toggleClaw = true;
                claw = !claw;
            }
            if (!gamepad1.triangle) toggleClaw = false;

            // square = elbow
            if (gamepad1.square && !toggleElbow) {
                toggleElbow = true;
                elbow = !elbow;
            }
            if (!gamepad1.square) toggleElbow = false;

            if (claw)
                robot.claw.setPosition(clawClosed);
            else
                robot.claw.setPosition(clawOpen);

            if (elbow)
                robot.elbow.setPosition(elbowUp);
            else
                robot.elbow.setPosition(elbowDown);

            ///////////////////
            // LIFT CONTROLS //
            ///////////////////

            // Left Lift
            //  - Square: negative power
            //  - Circle: positive power
            //
            // Right Lift
            //  - Cross: negative power
            //  - Trngl: positive power
            //
            // Both Motors
            //  - Left Trigger: negative power
            //  - Right Trigger: positive power

            double leftPower = 0;
            double rightPower = 0;

            // Read gamepad
            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            leftPower += triggerPower;
            rightPower += triggerPower;

            // Clamp powers
            leftPower = clamp(leftPower);
            rightPower = clamp(rightPower);

            // Set powers
            robot.leftLift.set(leftPower);
            robot.rightLift.set(rightPower);

        }

    }

    /**
     * Clamp the input power between [-1,1]
     * @return Math.max(-1, Math.min(1, x));
     */
    private double clamp(double power) {
        return Math.max(-1, Math.min(1, power));
    }

}
