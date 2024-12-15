package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;

public class HardwareRobot {
    public final MotorEx leftFront;
    public final MotorEx rightFront;
    public final MotorEx leftBack;
    public final MotorEx rightBack;

    public final Servo leftDepositLift;
    public final Servo rightDepositLift;
    public final Servo leftIntakeLift;
    public final Servo rightIntakeLift;

    public final GoBildaPinpointDriver pinpoint;

    public final Limelight3A limelight;

    public final Servo intakePivot;
    public final Servo intakeWrist;
    public final Servo intakeClaw;
    public final Servo depositWrist;
    public final Servo depositClaw;
    public final Servo clipIntake;
    public final Servo clipPusher;


    public HardwareRobot(HardwareMap hardwareMap) {

        ////////////
        // WHEELS //
        ////////////
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

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(true);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //////////
        // LIFT //
        //////////
        leftIntakeLift = hardwareMap.get(ServoImplEx.class, "leftIntakeLift");
        rightIntakeLift = hardwareMap.get(ServoImplEx.class, "rightIntakeLift");
        leftDepositLift = hardwareMap.get(ServoImplEx.class, "leftDepositLift");
        rightDepositLift = hardwareMap.get(ServoImplEx.class, "rightDepositLift");

//        leftIntakeLift = new MotorEx(hardwareMap, "leftIntakeLift", Motor.GoBILDA.RPM_312);
//        leftIntakeLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftIntakeLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftIntakeLift.setRunMode(Motor.RunMode.RawPower);
//        leftIntakeLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftIntakeLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        leftIntakeLift.setInverted(false);
//
//        rightDepositLift = new MotorEx(hardwareMap, "rightDepositLift", Motor.GoBILDA.RPM_312);
//        rightDepositLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightDepositLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightDepositLift.setRunMode(Motor.RunMode.RawPower);
//        rightDepositLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDepositLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        rightDepositLift.setInverted(true);
//
//        leftDepositLift = new MotorEx(hardwareMap, "leftDepositLift", Motor.GoBILDA.RPM_312);
//        leftDepositLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftDepositLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftDepositLift.setRunMode(Motor.RunMode.RawPower);
//        leftDepositLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftDepositLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        leftDepositLift.setInverted(false);
//
//        rightIntakeLift = new MotorEx(hardwareMap, "rightIntakeLift", Motor.GoBILDA.RPM_312);
//        rightIntakeLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightIntakeLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightIntakeLift.setRunMode(Motor.RunMode.RawPower);
//        rightIntakeLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightIntakeLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        rightIntakeLift.setInverted(true);


        //////////////
        // PINPOINT //
        //////////////
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        ////////////
        // SERVOS //
        ////////////
        intakePivot = hardwareMap.get(ServoImplEx.class, "intakePivot");
        intakeWrist = hardwareMap.get(ServoImplEx.class, "intakeWrist");
        intakeClaw = hardwareMap.get(ServoImplEx.class, "intakeClaw");
        depositClaw = hardwareMap.get(ServoImplEx.class, "depositClaw");
        depositWrist = hardwareMap.get(ServoImplEx.class, "depositWrist");
        clipIntake = hardwareMap.get(ServoImplEx.class, "clipIntake");
        clipPusher = hardwareMap.get(ServoImplEx.class, "clipPusher");


        ////////////
        // CAMERA //
        ////////////
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

    }
}