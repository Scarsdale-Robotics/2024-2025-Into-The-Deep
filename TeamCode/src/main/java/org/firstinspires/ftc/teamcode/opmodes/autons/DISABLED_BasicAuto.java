package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;

@Disabled
@Autonomous(name="Basic Auto")
public class DISABLED_BasicAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack
        );

        waitForStart();

        double speed = 0.15;
        while (opModeIsActive()) {
            telemetry.addData("MOVING", "FORWARD");
            telemetry.update();
            drive.driveRobotCentric(0, speed, 0);
            sleep(1000);

            telemetry.addData("MOVING", "STOP");
            telemetry.update();
            drive.driveRobotCentric(0, 0, 0);
            sleep(1000);

            telemetry.addData("MOVING", "BACKWARD");
            telemetry.update();
            drive.driveRobotCentric(0, -speed, 0);
            sleep(1000);

            telemetry.addData("MOVING", "STOP");
            telemetry.update();
            drive.driveRobotCentric(0, 0, 0);
            sleep(1000);
        }
    }
}
