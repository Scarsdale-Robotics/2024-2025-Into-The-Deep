package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareRobot;

@TeleOp(name = "Static Friction Calibrator", group = "Calibration")
public class StaticFrictionCalibrator extends LinearOpMode {
    public MotorEx leftFront;
    public MotorEx rightFront;
    public MotorEx leftBack;
    public MotorEx rightBack;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftFront = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftBack = new MotorEx(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        rightBack = new MotorEx(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);
        MecanumDrive drive = new MecanumDrive(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );

        waitForStart();

        double forwardPower = 0;
        boolean toggleTriangle = false;
        boolean toggleCross = false;
        while (opModeIsActive()) {
            // Increment by 0.01
            if (gamepad1.triangle && !toggleTriangle) {
                forwardPower += 0.01;
                toggleTriangle = true;
            }
            if (!gamepad1.triangle) toggleTriangle = false;

            // Decrement by 0.01
            if (gamepad1.cross && !toggleCross) {
                forwardPower -= 0.01;
                toggleCross = true;
            }
            if (!gamepad1.cross) toggleCross = false;

            // Telemetry
            telemetry.addData("FORWARD POWER", forwardPower);
            telemetry.update();

            // Drive
            drive.driveRobotCentric(0, forwardPower, 0);

        }

    }
}
