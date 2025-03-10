package org.firstinspires.ftc.teamcode.opmodes.calibration.outtake_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


@Config
@TeleOp(name = "Lift Gravity Calibrator", group = "Calibration")
public class LiftGravityCalibrator extends LinearOpMode {

    private MotorEx leftLift;
    private MotorEx rightLift;

    public static double power = 0.0;

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
        leftLift.setInverted(false);

        rightLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setRunMode(Motor.RunMode.RawPower);
        rightLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLift.setInverted(true);

        telemetry.addData("[TICKS] Left", 0);
        telemetry.addData("[TICKS] Right", 0);
        telemetry.addData("[POWER] Left", 0);
        telemetry.addData("[POWER] Right", 0);
        telemetry.addData("[VELOCITY] Left", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            leftLift.motor.setPower(power);
            rightLift.motor.setPower(power);

            // Telemetry
            telemetry.addData("[TICKS] Left", leftLift.getCurrentPosition());
            telemetry.addData("[TICKS] Right", rightLift.getCurrentPosition());
            telemetry.addData("[POWER]", power);
            telemetry.update();
        }

    }


}
