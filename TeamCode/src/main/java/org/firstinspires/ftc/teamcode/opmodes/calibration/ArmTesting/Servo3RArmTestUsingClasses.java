package org.firstinspires.ftc.teamcode.opmodes.calibration.ArmTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Config
@TeleOp(name="3R Arm Test Using Classes", group="Calibration")
public class Servo3RArmTestUsingClasses extends LinearOpMode {

    private Servo servoArmBase;
    private Servo servoArmFirstJoint;
    private Servo servoArmSecondJoint;
    private WebcamName cameraName;

    public static double targetX = 5; // +X is forward
    public static double targetY = 0; // +Y is left
    public static double targetZ = 0; // +Z is upward

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        servoArmBase = hardwareMap.get(ServoImplEx.class, "armBase");
        servoArmFirstJoint = hardwareMap.get(ServoImplEx.class, "armFirstJoint");
        servoArmSecondJoint = hardwareMap.get(ServoImplEx.class, "armSecondJoint");
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        Servo3RArmIK servo3RArmIK = new Servo3RArmIK();
        Servo3RArmController servo3RArmController = new Servo3RArmController(
                servoArmBase,
                servoArmFirstJoint,
                servoArmSecondJoint
        );

        waitForStart();

        while (opModeIsActive()) {
            servo3RArmController.setPosition(servo3RArmIK.getServoArmPositions(
                    new double[]{targetX, targetY, targetZ}
            ));
        }
    }

}
