package org.firstinspires.ftc.teamcode.opmodes.calibration.NewRobotTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Autonomous(name="Extendo Logger", group = "Calibration")
public class ExtendoLogger extends LinearOpMode {

    private Motor extendo;
    public static double SPEED = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();
        waitForStart();

        while (opModeIsActive()) {
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            power *= SPEED;

            extendo.motor.setPower(power);

            telemetry.addData("power", power);
            telemetry.addData("position", extendo.getCurrentPosition());
            telemetry.update();
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extendo = new MotorEx(hardwareMap, "extendo", Motor.GoBILDA.RPM_312);
        extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setRunMode(Motor.RunMode.RawPower);
        extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendo.setInverted(false);
    }
}
