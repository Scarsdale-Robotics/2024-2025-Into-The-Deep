package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


@Config
@TeleOp(name = "Lift Encoder Logger", group = "Calibration")
public class LiftEncoderLogger extends LinearOpMode {

    private MotorEx leftLift;
    private MotorEx rightLift;

    public static double kP = 0.01;
    public static double kI = 0.01;
    public static double kD = 0;
    private ArrayList<Double> lastErrors = new ArrayList<>();
    private ArrayList<Double> deltaTimes = new ArrayList<>();
    private double errorSum = 0;
    private ElapsedTime runtime;

    public static double speed = 0.5;

    public static double joyspeed = 10;
    private double encoderSetpoint = 0;
    public static double maxTicks = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftLift = new MotorEx(hardwareMap, "leftLift", Motor.GoBILDA.RPM_312);
        rightLift = new MotorEx(hardwareMap, "rightLift", Motor.GoBILDA.RPM_312);

        leftLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setRunMode(Motor.RunMode.RawPower);
        leftLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftLift.setInverted(false);

        rightLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setRunMode(Motor.RunMode.RawPower);
        rightLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLift.setInverted(true);

        telemetry.addData("[TICKS] Left", 0);
        telemetry.addData("[TICKS] Right", 0);
        telemetry.addData("[POWER] Left", 0);
        telemetry.addData("[POWER] Right", 0);
        telemetry.addData("[VELOCITY] Left", 0);
        telemetry.update();

        runtime = new ElapsedTime(0);
        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {
            // CONTROLS
            //
            // Left Lift
            //  - Square: negative power
            //  - Circle: positive power
            //
            // Right Lift
            //  - Cross: negative power
            //  - Trngl: positive power
            //
            // Both Motors
            //  - Left Trigger: negative power
            //  - Right Trigger: positive power

            double leftPower = 0;
            double rightPower = 0;

            // Read gamepad
            if (gamepad1.square) leftPower -= 1;
            if (gamepad1.circle) leftPower += 1;

            if (gamepad1.cross) rightPower -= 1;
            if (gamepad1.triangle) rightPower += 1;

            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            leftPower += triggerPower;
            rightPower += triggerPower;

            // Clamp powers
            leftPower = clamp(leftPower);
            rightPower = clamp(rightPower);





            // Set powers
            leftLift.set(speed*leftPower);
            if (gamepad1.right_bumper) {
                double deltaTime = runtime.seconds();
                // setpoint
                encoderSetpoint += -gamepad1.left_stick_y*joyspeed*deltaTime;
                encoderSetpoint = Math.min(Math.max(0, encoderSetpoint),maxTicks);
                // PID controller
                double pv = rightLift.getCurrentPosition();
                double e = encoderSetpoint - pv;
                runtime.reset();

                // de/dt
                lastErrors.add(e);
                deltaTimes.add(deltaTime);
                double derivative = 0;
                if (lastErrors.size() > 5) {
                    lastErrors.remove(0);
                    deltaTimes.remove(0);
                    double avgDeltaTime = deltaTimes.stream()
                            .mapToDouble(d -> d)
                            .average()
                            .orElse(0.0);
                    derivative = (-lastErrors.get(0)+8*lastErrors.get(1)-8*lastErrors.get(3)+lastErrors.get(4))/
                            (12*avgDeltaTime);
                }

                // int edt
                errorSum += deltaTime*e;
                if (lastErrors.size()>1) {
                    if (lastErrors.get(lastErrors.size()-2)*e <= 0) {
                        // flush integral stack
                        errorSum = 0;
                    }
                }

                double ut = kP*e + kI*errorSum + kD*derivative;
                rightLift.set(ut);
                telemetry.addData("error", e);
                telemetry.addData("errorSum", errorSum);
                telemetry.addData("errorDerivative", derivative);
                telemetry.addData("ut", ut);
            } else {
                rightLift.set(speed*rightPower);
            }

            // Telemetry
            telemetry.addData("[TICKS] Left", leftLift.getCurrentPosition());
            telemetry.addData("[TICKS] Right", rightLift.getCurrentPosition());
            telemetry.addData("[POWER] Left", leftPower);
            telemetry.addData("[POWER] Right", rightPower);
            telemetry.addData("[TICKS] setpoint", encoderSetpoint);
            telemetry.update();
        }

    }

    /**
     * Clamp the input power between [-1,1]
     * @return Math.max(-1, Math.min(1, x));
     */
    private double clamp(double power) {
        return Math.max(-1, Math.min(1, power));
    }

}
