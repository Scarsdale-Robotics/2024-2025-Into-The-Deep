package org.firstinspires.ftc.teamcode.opmodes.calibration.intake_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name="Horizontal Intake Calibrator", group="Calibration")
public class HorizontalIntakeCalibrator extends LinearOpMode {

    private Servo leftHorizontalArm;
    private Servo rightHorizontalArm;
    private Servo horizontalWrist;
    private Servo horizontalClaw;

    public static double armLeftZeroPosition = 0.85;
    public static double armLeftPiPosition = 0.15;

    public static double armRightZeroPosition = 0.16;
    public static double armRightPiPosition = 0.86;

    public static double horizontalArmPosition = 0.5;
    public static double horizontalWristPosition = 0.5;
    public static double horizontalClawPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftHorizontalArm = hardwareMap.get(ServoImplEx.class, "leftHorizontalArm");
        rightHorizontalArm = hardwareMap.get(ServoImplEx.class, "rightHorizontalArm");
        horizontalWrist = hardwareMap.get(ServoImplEx.class, "horizontalWrist");
        horizontalClaw = hardwareMap.get(ServoImplEx.class, "horizontalClaw");

        waitForStart();

        while (opModeIsActive()) {
            double[] armServoPositions = toArmServoPositions(horizontalArmPosition);
            leftHorizontalArm.setPosition(armServoPositions[0]);
            rightHorizontalArm.setPosition(armServoPositions[1]);
            telemetry.addData("leftHorizontalArmPosition", armServoPositions[0]);
            telemetry.addData("rightHorizontalArmPosition", armServoPositions[1]);
            telemetry.update();

            horizontalWrist.setPosition(horizontalWristPosition);
            horizontalClaw.setPosition(horizontalClawPosition);
        }

    }

    /**
     * @param horizontalArmPosition between [0, 1] where 0 is at spec maker and 1 is on the ground
     * @return {leftHorizontalArmPosition, rightHorizontalArmPosition}
     */
    private double[] toArmServoPositions(double horizontalArmPosition) {
        double leftHorizontalArmPosition = armLeftZeroPosition + horizontalArmPosition*(armLeftPiPosition - armLeftZeroPosition);
        double rightHorizontalArmPosition = armRightZeroPosition + horizontalArmPosition*(armRightPiPosition - armRightZeroPosition);
        return new double[]{leftHorizontalArmPosition, rightHorizontalArmPosition};
    }
}
