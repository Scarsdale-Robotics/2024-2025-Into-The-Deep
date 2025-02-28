package org.firstinspires.ftc.teamcode.opmodes.calibration.deposit_testing;

import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;

@Config
@TeleOp(name = "Deposit Servo Calibrator", group = "Calibration")
public class DepositServoCalibrator extends LinearOpMode {
    public static double D_ARM_DEPOSIT = 0.;
    public static double D_ARM_TRANSFER = 0.;
    public static double D_ARM_SPEC_MAKER = 0.;
    public static double D_CLAW_OPEN = 0.;
    public static double D_CLAW_CLOSED = 0.;  // consider 2 closed positions
                                              // for extra tightness before deposit
                                              // and decreased tightness after deposit

    public static double armPos = VArmConstants.armLeftClipperPosition;
    public static double clawPos = VClawConstants.GRAB_POSITION;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftArm = initServo(hardwareMap, "leftDepositArm");
        Servo rightArm = initServo(hardwareMap, "rightDepositArm");
        Servo depositClaw = initServo(hardwareMap, "depositClaw");

        waitForStart();

        while (opModeIsActive()) {
            leftArm.setPosition(armPos);
            rightArm.setPosition(armPos-0.15);
            depositClaw.setPosition(clawPos);
        }
    }
}
