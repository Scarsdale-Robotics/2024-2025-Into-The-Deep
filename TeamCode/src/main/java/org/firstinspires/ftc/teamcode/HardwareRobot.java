package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class HardwareRobot {
    public final MotorEx leftFront;
    public final MotorEx rightFront;
    public final MotorEx leftBack;
    public final MotorEx rightBack;

    public final MotorEx leftArm;
    public final MotorEx rightArm;

    public final ServoImplEx elbow;
    public final Servo leftClaw;
    public final Servo rightClaw;
    public final ServoImplEx wrist;

    public final AdafruitBNO055IMU imu;

    public HardwareRobot(HardwareMap hardwareMap) {
        leftFront = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftBack = new MotorEx(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        rightBack = new MotorEx(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(false);
        rightFront.setInverted(false);
        leftBack.setInverted(false);
        rightBack.setInverted(false);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftArm = new MotorEx(hardwareMap, "leftArm", Motor.GoBILDA.RPM_312);
        leftArm.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setRunMode(Motor.RunMode.VelocityControl);
        leftArm.resetEncoder();
        leftArm.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftArm.setPositionTolerance(10);
        leftArm.setPositionCoefficient(0.01);

        rightArm = new MotorEx(hardwareMap, "rightArm", Motor.GoBILDA.RPM_312);
        rightArm.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setRunMode(Motor.RunMode.VelocityControl);
        rightArm.resetEncoder();
        rightArm.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightArm.setPositionTolerance(10);
        rightArm.setPositionCoefficient(0.01);

        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        elbow = hardwareMap.get(ServoImplEx.class, "elbow");
        elbow.resetDeviceConfigurationForOpMode();

        leftClaw.scaleRange(0, 1);
        rightClaw.scaleRange(0, 1);
        wrist.scaleRange(0, 1);
        elbow.scaleRange(0, 1);

        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        elbow.setPwmRange(new PwmControl.PwmRange(500, 2500));

        imu = hardwareMap.get(AdafruitBNO055IMU.class, "imu");
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.resetDeviceConfigurationForOpMode();
        imu.initialize(new BNO055IMU.Parameters());

    }
}