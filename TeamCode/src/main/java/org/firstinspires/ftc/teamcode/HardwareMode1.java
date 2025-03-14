package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//creating different hardawarerobot systems and changing robotsystem for different initializations
//part of transition between modes
public class HardwareMode1 {
    //motors for default drive, odometry, cv mode
    //separate hardwarerobot classes will be added for other modes.
        public final MotorEx leftFront;
        public final MotorEx rightFront;
        public final MotorEx leftBack;
        public final MotorEx rightBack;
        public HardwareMode1(HardwareMap hardwareMap) {
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

        }
}
