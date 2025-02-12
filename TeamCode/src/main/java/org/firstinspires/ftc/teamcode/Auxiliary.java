package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Auxiliary {
    public static Motor initMotor(HardwareMap hardwareMap, String id, Motor.GoBILDA type) {
        Motor motor = new MotorEx(hardwareMap, id, type);
        motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public static Servo initServo(HardwareMap hardwareMap, String id) {
        return hardwareMap.get(ServoImplEx.class, id);
    }
}
