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
    Servo specimenDropper;

    public static double specimenClipperDeactivatedPosition = 0;
    public static double specimenClipperActivatedPosition = 0.3;

    public static double specimenDropperDeactivatedPosition = 0.5;
    public static double specimenDropperActivatedPosition = 0.7;

    public int specimenMakerState = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        specimenClipper = hardwareMap.get(ServoImplEx.class, "specimenClipper");
        double specimenClipperPosition = 0;

        specimenDropper = hardwareMap.get(ServoImplEx.class, "specimenDropper");
        double specimenDropperPosition = 0;

        waitForStart();

        boolean rbPressed = false;
        boolean lbPressed = false;
        int n_states = 4;
        while (opModeIsActive()) {

            // LB = Go back state
            // RB = Advance state

            if (gamepad1.left_bumper && !lbPressed) {
                lbPressed = true;
            } else if (!gamepad1.left_bumper & lbPressed) {
                lbPressed = false;
                specimenMakerState = specimenMakerState>0 ? ((specimenMakerState - 1)%n_states) : 0;
            }

            if (gamepad1.right_bumper && !rbPressed) {
                rbPressed = true;
            } else if (!gamepad1.right_bumper & rbPressed) {
                rbPressed = false;
                specimenMakerState = (specimenMakerState + 1)%n_states;
            }



            // Control servos

            if (specimenMakerState == 0) {
                specimenDropperPosition = specimenDropperDeactivatedPosition;
                specimenClipperPosition = specimenClipperDeactivatedPosition;
            }
            if (specimenMakerState == 1) {
                specimenDropperPosition = specimenDropperActivatedPosition;
                specimenClipperPosition = specimenClipperDeactivatedPosition;
            }
            if (specimenMakerState == 2) {
                specimenDropperPosition = specimenDropperActivatedPosition;
                specimenClipperPosition = specimenClipperActivatedPosition;
            }
            if (specimenMakerState == 3) {
                specimenDropperPosition = specimenDropperActivatedPosition;
                specimenClipperPosition = specimenClipperDeactivatedPosition;
            }

            specimenClipper.setPosition(specimenClipperPosition);
            specimenDropper.setPosition(specimenDropperPosition);


        }

    }
}
