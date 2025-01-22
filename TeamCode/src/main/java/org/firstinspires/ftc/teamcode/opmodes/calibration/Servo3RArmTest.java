package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Servo3RArmTest extends LinearOpMode {

    private Servo servoArmBase;
    private Servo servoArmFirstJoint;
    private Servo servoArmSecondJoint;
    private WebcamName cameraName;

    private double armBaseZero = 0.53;
    private double armBaseNinety = 0.85;

    private double armFirstJointZero = 0.34;
    private double armFirstJointNinety = 0.66;

    private double armSecondJointZero = 0.51;
    private double armSecondJointNinety = 0.21;

    private double armFirstSegmentLength = 9;
    private double armSecondSegmentLength = 9;

    @Override
    public void runOpMode() throws InterruptedException {
        servoArmBase = hardwareMap.get(ServoImplEx.class, "armBase");
        servoArmFirstJoint = hardwareMap.get(ServoImplEx.class, "armFirstJoint");
        servoArmSecondJoint = hardwareMap.get(ServoImplEx.class, "armSecondJoint");
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");



    }
}
