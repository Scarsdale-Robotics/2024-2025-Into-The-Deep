package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Lift Encoder Logger", group = "Calibration")
public class LiftEncoderLogger extends LinearOpMode {

    private MotorEx leftLift;
    private MotorEx rightLift;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftLift = new MotorEx(hardwareMap, "leftLift", Motor.GoBILDA.RPM_312);
        rightLift = new MotorEx(hardwareMap, "rightLift", Motor.GoBILDA.RPM_312);

        leftLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setRunMode(Motor.RunMode.RawPower);
        leftLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftLift.setInverted(true);

        rightLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setRunMode(Motor.RunMode.RawPower);
        rightLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("[TICKS] Left", 0);
        telemetry.addData("[TICKS] Right", 0);
        telemetry.addData("[POWER] Left", 0);
        telemetry.addData("[POWER] Right", 0);
        telemetry.addData("[VELOCITY] Left", 0);
        telemetry.update();

        waitForStart();

        double lastPosition = 0;
        while (opModeIsActive()) {
            // CONTROLS
            //
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
            if (gamepad1.square) leftPower -= 1;
            if (gamepad1.circle) leftPower += 1;

            if (gamepad1.cross) rightPower -= 1;
            if (gamepad1.triangle) rightPower += 1;

            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            leftPower += triggerPower;
            rightPower += triggerPower;

            // Clamp powers
            leftPower = clamp(leftPower);
            rightPower = clamp(rightPower);

            // Set powers
            leftLift.set(leftPower);
            rightLift.set(rightPower);

            // Telemetry
            telemetry.addData("[TICKS] Left", leftLift.getCurrentPosition());
            telemetry.addData("[TICKS] Right", rightLift.getCurrentPosition());
            telemetry.addData("[POWER] Left", leftPower);
            telemetry.addData("[POWER] Right", rightPower);
            telemetry.update();
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
