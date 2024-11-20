package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;

@Config
@TeleOp(name="Claw Calibrator", group="Calibration")
public class ClawCalibrator extends LinearOpMode {

    public static double servoPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareRobot hardwareRobot = new HardwareRobot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            hardwareRobot.claw.setPosition(servoPosition);
        }
    }
}
