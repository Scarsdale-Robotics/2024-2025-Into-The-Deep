package org.firstinspires.ftc.teamcode.opmodes.calibration.deposit_testing;

import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;

@Config
@TeleOp(name = "Deposit RArm Servo Calibrator", group = "Calibration")
public class DepositRArmServoCalibrator extends LinearOpMode {

    public static double armPos = VArmConstants.armLeftClipperPosition + VArmConstants.SERVO_DIFFERENCE;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo rightArm = initServo(hardwareMap, "rightDepositArm");

        waitForStart();

        while (opModeIsActive()) {
            rightArm.setPosition(armPos);
        }
    }
}
