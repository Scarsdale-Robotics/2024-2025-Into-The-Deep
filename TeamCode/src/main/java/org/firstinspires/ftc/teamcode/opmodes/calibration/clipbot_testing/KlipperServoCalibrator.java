package org.firstinspires.ftc.teamcode.opmodes.calibration.clipbot_testing;

import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Klipper Servo Calibrator", group = "Calibration")
public class KlipperServoCalibrator extends LinearOpMode {
    public static double pos = 0.;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo klipper = initServo(hardwareMap, "klipper");

        waitForStart();

        while (opModeIsActive()) {
            klipper.setPosition(pos);
        }
    }
}
