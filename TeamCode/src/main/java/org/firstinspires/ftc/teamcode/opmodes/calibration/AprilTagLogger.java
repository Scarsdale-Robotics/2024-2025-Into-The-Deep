package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name="AprilTag Logger", group="Calibration")
public class AprilTagLogger extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack
        );
        CVSubsystem cv = new CVSubsystem(robot.limelight, Math.PI, telemetry);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        FtcDashboard.getInstance().startCameraStream();

        waitForStart();

        // apriltag-only KF
        double x = 0;
        double y = 0;
        double h = Math.PI;
        double P_translation = 4;
        double P_heading = 0.01;

        double speed = 0.3;
        while (opModeIsActive()) {
            /// TELEOP ///
            double forward = -speed * gamepad1.left_stick_y;
            double strafe = speed * gamepad1.left_stick_x;
            double turn = speed * gamepad1.right_stick_x;
            drive.driveRobotCentric(strafe, forward, turn);

            /// CV ///
            cv.telemeterAbsoluteAprilTags();

            CVSubsystem.PoseEstimation poseEstimation = cv.getPoseEstimation();

            if (poseEstimation!=null) {

                // "Predict" step (just increment P)
                double[] stddev = cv.getStddev();
                P_translation += stddev[0]*stddev[0];
                P_heading += stddev[1]*stddev[1];

                // Update step of KF
                double K_translation = P_translation / (P_translation + poseEstimation.translationCovariance);
                x += K_translation * (poseEstimation.pose.getX() - x);
                y += K_translation * (poseEstimation.pose.getY() - y);
                P_translation = (1 - K_translation) * P_translation;

                double K_heading = P_heading / (P_heading + poseEstimation.headingCovariance);
                h += K_heading * (normalizeAngle(poseEstimation.pose.getHeading() - h));
                h = normalizeAngle(h);
                P_heading = (1 - K_heading) * P_heading;


                // Draw robot on dashboard
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), poseEstimation.pose);
                Drawing.drawTargetPose(packet.fieldOverlay(), new Pose2d(x, y, new Rotation2d(h)));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                // update MT2 algo
                cv.getLimelight().updateRobotOrientation(Math.toDegrees(h));

                telemetry.addData("P_translation", P_translation);
                telemetry.addData("P_heading", P_heading);
                telemetry.addData("stddev[0]", stddev[0]);
                telemetry.addData("stddev[1]", stddev[1]);
                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("h", h);
                telemetry.update();
            }

        }

    }

    /**
     * Normalizes a given angle to (-pi,pi] radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private static double normalizeAngle(double radians) {
        return (radians + Math.PI) % (2*Math.PI) - Math.PI;
    }
}
