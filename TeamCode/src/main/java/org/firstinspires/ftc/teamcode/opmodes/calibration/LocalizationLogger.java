package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name="Localization Logger", group="Calibration")
public class LocalizationLogger extends LinearOpMode {

    @Override
    public void runOpMode() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        RobotSystem robot = new RobotSystem(hardwareMap, new Pose2d(60, 24, new Rotation2d(Math.PI/2)), this);
        RobotSystem robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)), false, this);

        robot.localization.disableCamera();

        waitForStart();

        double speed = 1;
        while (opModeIsActive()) {
            robot.logOdometry();
            // Draw robot on dashboard
            Pose2d currentPose = robot.localization.getPose();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), currentPose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("LocalizationLogger X", currentPose.getX());
            telemetry.addData("LocalizationLogger Y", currentPose.getY());
            telemetry.addData("LocalizationLogger H", Math.toDegrees(currentPose.getHeading()) + "°");
            telemetry.update();

            double forward = -speed*gamepad1.left_stick_y;
            double strafe = speed*gamepad1.left_stick_x;
            double turn = speed*gamepad1.right_stick_x;

//            double kP = 4;
//            turn -= kP*(0-currentPose.getHeading());


//
//            double theta;
//            double spd = Math.hypot(strafe, forward) * DriveConstants.MAX_VELOCITY;
//            if (spd==0) theta = 0;
//            else theta = Math.atan2(forward, strafe);
//
//            telemetry.addData("THETA", theta);
//            telemetry.addData("SPEED", spd);
//            telemetry.addData("TURN", -turn * DriveConstants.MAX_ANGULAR_VELOCITY);
//
//
//
//            double maxSpeed = Math.hypot(
//                    DriveConstants.MAX_STRAFE_SPEED*Math.cos(theta),
//                    DriveConstants.MAX_FORWARD_SPEED*Math.sin(theta)
//            );
//
//            double L = 0;
//            double R = 0;
//            double theta_w = DriveConstants.THETA_WHEEL;
//            if (0<theta && theta<=Math.PI/2) {
//                L = 1;
//                R = -Math.sin(theta_w-theta) / Math.sin(theta_w+theta);
//            }
//            else if (Math.PI/2<theta && theta<=Math.PI) {
//                L = -Math.sin(theta_w+theta) / Math.sin(theta_w-theta);
//                R = 1;
//            }
//            else if (-Math.PI<=theta && theta<=-Math.PI/2) {
//                L = -1;
//                R = Math.sin(theta_w-theta) / Math.sin(theta_w+theta);
//            }
//            else if (-Math.PI/2<theta && theta<=0) {
//                L = Math.sin(theta_w+theta) / Math.sin(theta_w-theta);
//                R = -1;
//            }
//
//            double factor = Math.max(-1, Math.min(1, spd / maxSpeed));
//            L *= factor;
//            R *= factor;
//
//            telemetry.addData("L", L);
//            telemetry.addData("R", R);



            robot.drive.driveRobotCentricPowers(strafe, forward, turn);


            telemetry.update();


        }

    }

}
