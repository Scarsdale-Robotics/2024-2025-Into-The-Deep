package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="Specimen Maker Calib", group="Calibration")
@Config
public class SpecimenMakerTest extends LinearOpMode {

    Servo specimenClipper;
    public static double servoPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        specimenClipper = hardwareMap.get(ServoImplEx.class, "specimenClipper");
        servoPosition = 0;

        waitForStart();

        while (opModeIsActive()) {
            specimenClipper.setPosition(servoPosition);
        }

    }
}
