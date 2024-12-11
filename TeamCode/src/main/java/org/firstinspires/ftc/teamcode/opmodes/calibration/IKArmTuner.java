package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="2R IK Arm Tuner", group="Calibration")
@Config
public class IKArmTuner extends LinearOpMode {

    Servo armBeta;
    Servo armGamma;

    public static double servoArmBetaPosition;
    public static double servoArmGammaPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        armBeta = hardwareMap.get(ServoImplEx.class, "armBeta");
        armGamma = hardwareMap.get(ServoImplEx.class, "armGamma");

        servoArmBetaPosition = 0;
        servoArmGammaPosition = 0;

        waitForStart();

        while (opModeIsActive()) {
            armBeta.setPosition(servoArmBetaPosition);
            armGamma.setPosition(servoArmGammaPosition);
        }

    }
}
