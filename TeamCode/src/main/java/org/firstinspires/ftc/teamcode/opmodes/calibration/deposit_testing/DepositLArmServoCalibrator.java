package org.firstinspires.ftc.teamcode.opmodes.calibration.deposit_testing;

import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;

@Config
@TeleOp(name = "Deposit LArm Servo Calibrator", group = "Calibration")
public class DepositLArmServoCalibrator extends LinearOpMode {

    public static double armPos = VArmConstants.armLeftClipperPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftArm = initServo(hardwareMap, "leftDepositArm");

        waitForStart();

        while (opModeIsActive()) {
            leftArm.setPosition(armPos);
        }
    }
}
