package org.firstinspires.ftc.teamcode.opmodes.calibration.clipbot_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name="Clipbot Servo Calibrator", group="Calibration")
public class ClipbotServoCalibrator extends LinearOpMode {

    private Servo magazineIntake;
    private Servo magazineLoader1;
    private Servo magazineLoader2;

    public static double intakeOutPosition = 0.5;
    public static double intakeInPosition = 0.5;

    public static double loaderUpPosition = 0;
    public static double loaderDownPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        waitForStart();

        boolean toggleTriangle = false,
                intakeIsIn = false;
        boolean toggleSquare = false,
                loaderIsDown = false;
        while (opModeIsActive()) {

            // toggle switches
            // - triangle = intake
            // - square = loader

            if (gamepad1.triangle && !toggleTriangle) {
                intakeIsIn = !intakeIsIn;
                toggleTriangle = true;
            } else if (!gamepad1.triangle) {
                toggleTriangle = false;
            } // out is open, in is closed

            if (gamepad1.square && !toggleSquare) {
                loaderIsDown = !loaderIsDown;
                toggleSquare = true;
            } else if (!gamepad1.square) {
                toggleSquare = false;
            }


            // control servos
            if (intakeIsIn) {
                magazineIntake.setPosition(intakeInPosition);
                telemetry.addData("intake position", "IN");
            } else {
                magazineIntake.setPosition(intakeOutPosition);
                telemetry.addData("intake position", "OUT");
            }

            if (loaderIsDown) {
                magazineLoader1.setPosition(loaderDownPosition);
                magazineLoader2.setPosition(1-loaderDownPosition);
                telemetry.addData("loader position", "DOWN");
            } else {
//                magazineLoader1.setPosition(loaderUpPosition);
//                magazineLoader2.setPosition(1-loaderUpPosition);
                telemetry.addData("loader position", "UP");
                telemetry.addData("loader 1 position", magazineLoader1.getPosition());
                telemetry.addData("loader 2 position", magazineLoader2.getPosition());
            }

            telemetry.update();
        }

    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        magazineIntake = hardwareMap.get(ServoImplEx.class, "magazineIntake");
        magazineLoader1 = hardwareMap.get(ServoImplEx.class, "magazineLoader1");
        magazineLoader2 = hardwareMap.get(ServoImplEx.class, "magazineLoader2");
    }
}
