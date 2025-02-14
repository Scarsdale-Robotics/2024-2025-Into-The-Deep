package org.firstinspires.ftc.teamcode.opmodes.calibration.static_friction_tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Config
@TeleOp(name="Extendo Static Friction Tuner", group="Calibration")
public class ExtendoStaticFrictionTuner extends LinearOpMode {

    public static double kS = 0;

    private Motor extendo;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        waitForStart();

        while (opModeIsActive()) {
            extendo.motor.setPower(kS);
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extendo = new MotorEx(hardwareMap, "extendo", Motor.GoBILDA.RPM_1620);
        extendo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setRunMode(Motor.RunMode.RawPower);
        extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendo.setInverted(true);
    }
}
