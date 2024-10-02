package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;

@TeleOp(name="Odometry Logger", group="Calibration")
public class OdometryLogger extends LinearOpMode {
    // Distance between parallel odometers
    private static final double TRACK_WIDTH = 11.3386; //TODO: tune
    // Signed distance from the point of rotation (positive=forward)
    private static final double CENTER_WHEEL_OFFSET = 0.944882; //TODO: tune
    // Measured in inches
    private static final double WHEEL_DIAMETER = 1.37795;
    // Odometer encoder resolution
    private static final double TICKS_PER_REV = 4096;
    // Circumference divided by encoder resolution [inches/tick]
    private static final double DISTANCE_PER_PULSE = (Math.PI * WHEEL_DIAMETER) / TICKS_PER_REV;

    // Encoders
    private Motor.Encoder leftOdometer;
    private Motor.Encoder rightOdometer;
    private Motor.Encoder centerOdometer;


    @Override
    public void runOpMode() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack
        );


        leftOdometer = robot.leftOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = robot.rightOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = robot.centerOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);

        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();
        HolonomicOdometry odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        waitForStart();

        Pose2d lastPose = new Pose2d();
        ElapsedTime runtime = new ElapsedTime(0);

        double maxSpeed = 0;
        double maxAngularSpeed = 0;

        double speed = 1;
        while (opModeIsActive()) {

            double forward = -speed*gamepad1.left_stick_y;
            double strafe = speed*gamepad1.left_stick_x;
            double turn = speed*gamepad1.right_stick_x;

            drive.driveRobotCentric(strafe, forward, turn);

            odometry.updatePose();

            // Draw robot on dashboard
            Pose2d currentPose = odometry.getPose();
            currentPose = new Pose2d(currentPose.getY(), currentPose.getX(), new Rotation2d(-currentPose.getHeading()));



            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), currentPose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("leftOdometer", leftOdometer.getPosition());
            telemetry.addData("centerOdometer", centerOdometer.getPosition());
            telemetry.addData("rightOdometer", rightOdometer.getPosition());
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("H", Math.toDegrees(currentPose.getHeading())+"deg");

            double currentSpeed = currentPose.minus(lastPose).getTranslation().getNorm()/runtime.seconds();
            maxSpeed = Math.max(maxSpeed, currentSpeed);
            telemetry.addData("maxSpeed", maxSpeed);
            telemetry.addData("currentSpeed", currentSpeed);

            double angularSpeed = Math.abs(currentPose.getHeading() - lastPose.getHeading())/runtime.seconds();
            maxAngularSpeed = Math.max(maxAngularSpeed, angularSpeed);
            telemetry.addData("maxAngularSpeed", maxAngularSpeed);
            telemetry.addData("currentAngularSpeed", angularSpeed);

            telemetry.update();

            runtime.reset();
            lastPose = currentPose;

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
