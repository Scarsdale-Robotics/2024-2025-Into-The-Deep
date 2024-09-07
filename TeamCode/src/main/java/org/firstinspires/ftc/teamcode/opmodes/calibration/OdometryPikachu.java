package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp(name="Odometry Pikachu", group="Calibration")
public class OdometryPikachu extends LinearOpMode {
    // Distance between parallel odometers
    private static final double TRACK_WIDTH = 6.649; //TODO: tune
    // Signed distance from the point of rotation (positive=forward)
    private static final double CENTER_WHEEL_OFFSET = 0; //TODO: tune
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

    private final double drive_speed = 0.5;

    public static double heavistep(double t) {
        if (t > 0) {
            return 1.0;
        } else if ( t < 0) {
            return 0.0;
        } else {
            return 0.5;
        }
    }

    public static double sgn(double t) {
        if (t > 0) {
            return 1.0;
        } else if (t < 0) {
            return -1.0;
        } else {
            return 0;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
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
        odometry.updatePose(new Pose2d(0, 0, new Rotation2d(0)));

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive()) {
            odometry.updatePose();
            Pose2d pose = odometry.getPose();
            pose = new Pose2d(-pose.getY(), pose.getX(), pose.getRotation());

            ////////////////////
            // DRIVE CONTROLS //
            ////////////////////
            double x = 0, y = 0, turn = 0;
            if (Math.abs(gamepad1.left_stick_x) > 0.05)
                x = gamepad1.left_stick_x;
            if (Math.abs(gamepad1.left_stick_y) > 0.05)
                y = -gamepad1.left_stick_y;
            if (Math.abs(gamepad1.right_stick_x) > 0.05)
                turn = gamepad1.right_stick_x;

            /////////////////////////
            // CORRECTION TO (0,0) //
            /////////////////////////

            double t = runtime.seconds(); // Period: 0 to 52pi
            double pi = Math.PI;
            double scale = 0.01;

            Pose2d target = new Pose2d(
                    ((-1.0/4 *Math.sin(10.0/7 - 23 *t) - 3.0/10 *Math.sin(4.0/3 - 22 *t) - 2.0/5 *Math.sin(7.0/5 - 19 *t) - 1.0/5 *Math.sin(7.0/5 - 16 *t) - 3.0/7 *Math.sin(10.0/7 - 15 *t) - 3.0/8 *Math.sin(13.0/9 - 9 *t) - 19.0/13 *Math.sin(11.0/7 - 3 *t) + 222.0/5 *Math.sin(t + 11.0/7) + 41.0/2 *Math.sin(2 *t + 11.0/7) + 34.0/9 *Math.sin(4 *t + 11.0/7) + 1.0/3 *Math.sin(5 *t + 8.0/5) + 3.0/8 *Math.sin(6 *t + 8.0/5) + 12.0/7 *Math.sin(7 *t + 13.0/8) + 11.0/7 *Math.sin(8 *t + 13.0/8) + 1.0/4 *Math.sin(10 *t + 20.0/13) + 2.0/9 *Math.sin(11 *t + 16.0/9) + 3.0/8 *Math.sin(12 *t + 8.0/5) + 1.0/3 *Math.sin(13 *t + 7.0/4) + 1.0/2 *Math.sin(14 *t + 17.0/10) + 5.0/7 *Math.sin(17 *t + 17.0/10) + 1.0/28 *Math.sin(18 *t + 9.0/2) + 1.0/2 *Math.sin(20 *t + 12.0/7) + 3.0/7 *Math.sin(21 *t + 16.0/9) + 6.0/11 *Math.sin(24 *t + 7.0/4) - 979.0/9) *heavistep(51 *pi - t) *heavistep(t - 47 *pi) + (-6.0/5 *Math.sin(14.0/9 - 22 *t) - 1.0/9 *Math.sin(7.0/5 - 19 *t) - 9.0/8 *Math.sin(14.0/9 - 18 *t) - 1.0/14 *Math.sin(15.0/11 - 15 *t) - 6.0/5 *Math.sin(11.0/7 - 12 *t) - 7.0/6 *Math.sin(11.0/7 - 8 *t) - 29.0/10 *Math.sin(11.0/7 - 6 *t) - 104.0/3 *Math.sin(11.0/7 - 2 *t) + 415.0/18 *Math.sin(t + 11.0/7) + 71.0/18 *Math.sin(3 *t + 11.0/7) + 19.0/8 *Math.sin(4 *t + 33.0/7) + 22.0/21 *Math.sin(5 *t + 8.0/5) + 3.0/8 *Math.sin(7 *t + 61.0/13) + 5.0/9 *Math.sin(9 *t + 11.0/7) + 1.0/8 *Math.sin(10 *t + 14.0/3) + 4.0/7 *Math.sin(11 *t + 11.0/7) + 4.0/11 *Math.sin(13 *t + 14.0/3) + 1.0/7 *Math.sin(14 *t + 14.0/3) + 2.0/7 *Math.sin(16 *t + 5.0/3) + 1.0/6 *Math.sin(17 *t + 5.0/3) + 6.0/7 *Math.sin(20 *t + 8.0/5) + 1.0/7 *Math.sin(21 *t + 5.0/3) + 1.0/6 *Math.sin(23 *t + 8.0/5) - 2765.0/8) *heavistep(47 *pi - t) *heavistep(t - 43 *pi) + (1189.0/22 *Math.sin(t + 11.0/7) + 3.0/4 *Math.sin(2 *t + 13.0/8) + 11.0/2 *Math.sin(3 *t + 8.0/5) + 2.0/7 *Math.sin(4 *t + 17.0/7) + 22.0/9 *Math.sin(5 *t + 18.0/11) + 1.0/4 *Math.sin(6 *t + 17.0/7) + 16.0/17 *Math.sin(7 *t + 20.0/11) + 1.0/5 *Math.sin(8 *t + 29.0/9) - 1627.0/7) *heavistep(43 *pi - t) *heavistep(t - 39 *pi) + (-3.0/7 *Math.sin(1.0/18 - 5 *t) - 3.0/4 *Math.sin(1.0/2 - 3 *t) + 109.0/9 *Math.sin(t + 13.0/10) + 5.0/8 *Math.sin(2 *t + 11.0/3) + 5.0/9 *Math.sin(4 *t + 10.0/3) + 3.0/10 *Math.sin(6 *t + 21.0/8) + 2.0/9 *Math.sin(7 *t + 2.0/3) + 1.0/4 *Math.sin(8 *t + 23.0/8) - 1190.0/9) *heavistep(39 *pi - t) *heavistep(t - 35 *pi) + (188.0/21 *Math.sin(t + 27.0/28) + 2.0/5 *Math.sin(2 *t + 17.0/6) + 2.0/3 *Math.sin(3 *t + 91.0/23) + 3.0/8 *Math.sin(4 *t + 53.0/18) + 2.0/11 *Math.sin(5 *t + 1.0/7) - 369) *heavistep(35 *pi - t) *heavistep(t - 31 *pi) + (-8.0/9 *Math.sin(1.0/10 - 12 *t) - 34.0/9 *Math.sin(10.0/9 - 6 *t) - 137.0/10 *Math.sin(5.0/7 - 2 *t) + 26.0/5 *Math.sin(t + 13.0/4) + 118.0/5 *Math.sin(3 *t + 11.0/8) + 43.0/8 *Math.sin(4 *t + 13.0/7) + 49.0/6 *Math.sin(5 *t + 11.0/12) + 22.0/5 *Math.sin(7 *t + 13.0/4) + 17.0/16 *Math.sin(8 *t + 1.0/7) + 5.0/4 *Math.sin(9 *t + 1.0/4) + 5.0/7 *Math.sin(10 *t + 17.0/5) + 29.0/15 *Math.sin(11 *t + 5.0/6) - 1915.0/8) *heavistep(31 *pi - t) *heavistep(t - 27 *pi) + (-2.0/7 *Math.sin(10.0/7 - 7 *t) - Math.sin(1.0/27 - 4 *t) + 68.0/7 *Math.sin(t + 44.0/15) + 76.0/9 *Math.sin(2 *t + 37.0/10) + 30.0/7 *Math.sin(3 *t + 1) + 8.0/9 *Math.sin(5 *t + 3.0/2) + 4.0/5 *Math.sin(6 *t + 31.0/8) + 3.0/7 *Math.sin(8 *t + 10.0/3) + 6.0/13 *Math.sin(9 *t + 8.0/7) + 1.0/3 *Math.sin(10 *t + 31.0/9) - 2135.0/9) *heavistep(27 *pi - t) *heavistep(t - 23 *pi) + (-3.0/8 *Math.sin(1.0/4 - 23 *t) - 3.0/5 *Math.sin(1.0/8 - 22 *t) - 13.0/8 *Math.sin(5.0/4 - 20 *t) - 9.0/7 *Math.sin(3.0/2 - 16 *t) - 41.0/5 *Math.sin(4.0/3 - 4 *t) + 768.0/7 *Math.sin(t + 11.0/5) + 109.0/5 *Math.sin(2 *t + 16.0/7) + 150.0/13 *Math.sin(3 *t + 11.0/6) + 33.0/7 *Math.sin(5 *t + 97.0/24) + 23.0/4 *Math.sin(6 *t + 5.0/7) + 69.0/7 *Math.sin(7 *t + 9.0/8) + 32.0/5 *Math.sin(8 *t + 21.0/5) + 7.0/6 *Math.sin(9 *t + 22.0/9) + 28.0/5 *Math.sin(10 *t + 5.0/6) + 43.0/10 *Math.sin(11 *t + 26.0/7) + 14.0/9 *Math.sin(12 *t + 5.0/11) + 13.0/9 *Math.sin(13 *t + 40.0/9) + 11.0/6 *Math.sin(14 *t + 2.0/5) + 3.0/2 *Math.sin(15 *t + 17.0/10) + 7.0/11 *Math.sin(17 *t + 4.0/3) + 3.0/8 *Math.sin(18 *t + 31.0/10) + 4.0/7 *Math.sin(19 *t + 14.0/9) + 6.0/5 *Math.sin(21 *t + 17.0/7) + 4.0/7 *Math.sin(24 *t + 27.0/8) + 1006.0/11) *heavistep(23 *pi - t) *heavistep(t - 19 *pi) + (-63.0/8 *Math.sin(2.0/7 - 8 *t) - 38.0/13 *Math.sin(11.0/9 - 6 *t) - 14.0/5 *Math.sin(1.0/17 - 4 *t) + 77.0/9 *Math.sin(t + 1.0/2) + 52.0/7 *Math.sin(2 *t + 10.0/3) + 22.0/9 *Math.sin(3 *t + 76.0/17) + 21.0/8 *Math.sin(5 *t + 26.0/7) + 3 *Math.sin(7 *t + 15.0/8) + 64.0/7 *Math.sin(9 *t + 57.0/14) + 6 *Math.sin(10 *t + 17.0/6) - 544.0/7) *heavistep(19 *pi - t) *heavistep(t - 15 *pi) + (-37.0/10 *Math.sin(4.0/7 - 5 *t) - 3 *Math.sin(3.0/7 - 3 *t) + 24.0/7 *Math.sin(t + 7.0/6) + 9.0/7 *Math.sin(2 *t + 2.0/5) + 31.0/15 *Math.sin(4 *t + 37.0/8) + 9.0/5 *Math.sin(6 *t + 12.0/5) + 59.0/12 *Math.sin(7 *t + 13.0/6) + 15.0/7 *Math.sin(8 *t + 25.0/8) + 134.0/15 *Math.sin(9 *t + 7.0/3) + 73.0/8 *Math.sin(10 *t + 1.0/5) - 4406.0/11) *heavistep(15 *pi - t) *heavistep(t - 11 *pi) + (236.0/7 *Math.sin(t + 6.0/5) + 1.0/2 *Math.sin(2 *t + 47.0/12) - 627.0/5) *heavistep(11 *pi - t) *heavistep(t - 7 *pi) + (69.0/2 *Math.sin(t + 5.0/6) - 715.0/2) *heavistep(7 *pi - t) *heavistep(t - 3 *pi) + (-19.0/9 *Math.sin(6.0/5 - 21 *t) - 37.0/10 *Math.sin(7.0/9 - 19 *t) - 23.0/8 *Math.sin(1 - 17 *t) - 16.0/3 *Math.sin(7.0/6 - 16 *t) - 29.0/5 *Math.sin(1.0/5 - 9 *t) - 919.0/11 *Math.sin(1.0/7 - 3 *t) + 1573.0/6 *Math.sin(t + 91.0/45) + 214.0/5 *Math.sin(2 *t + 33.0/8) + 421.0/14 *Math.sin(4 *t + 13.0/8) + 61.0/6 *Math.sin(5 *t + 19.0/5) + 401.0/16 *Math.sin(6 *t + 43.0/14) + 511.0/51 *Math.sin(7 *t + 35.0/8) + 144.0/7 *Math.sin(8 *t + 5.0/6) + 137.0/10 *Math.sin(10 *t + 25.0/13) + 18.0/7 *Math.sin(11 *t + 15.0/7) + 17.0/9 *Math.sin(12 *t + 41.0/9) + 9.0/7 *Math.sin(13 *t + 13.0/7) + 29.0/10 *Math.sin(14 *t + 22.0/7) + 25.0/8 *Math.sin(15 *t + 1.0/4) + 12.0/5 *Math.sin(18 *t + 11.0/8) + 14.0/5 *Math.sin(20 *t + 27.0/7) + 13.0/8 *Math.sin(22 *t + 12.0/7) + 7.0/6 *Math.sin(23 *t + 7.0/9) + 26.0/11 *Math.sin(24 *t + 23.0/7) - 1891.0/8) *heavistep(3 *pi - t) *heavistep(t + pi)) *heavistep(Math.sqrt(sgn(Math.sin(t/2))))
                    ,
                    ((-8.0/11 *Math.sin(11.0/8 - 22 *t) - 1.0/2 *Math.sin(10.0/7 - 21 *t) + 67.0/6 *Math.sin(t + 33.0/7) + 1478.0/29 *Math.sin(2 *t + 11.0/7) + 3.0/5 *Math.sin(3 *t + 30.0/7) + 26.0/3 *Math.sin(4 *t + 11.0/7) + 1.0/6 *Math.sin(5 *t + 13.0/9) + 30.0/29 *Math.sin(6 *t + 8.0/5) + 2.0/5 *Math.sin(7 *t + 14.0/3) + 88.0/29 *Math.sin(8 *t + 8.0/5) + 1.0/4 *Math.sin(9 *t + 31.0/7) + 11.0/8 *Math.sin(10 *t + 8.0/5) + 1.0/16 *Math.sin(11 *t + 9.0/2) + 1.0/12 *Math.sin(12 *t + 5.0/4) + 1.0/10 *Math.sin(13 *t + 25.0/11) + 11.0/8 *Math.sin(14 *t + 18.0/11) + 2.0/7 *Math.sin(15 *t + 37.0/8) + 1.0/6 *Math.sin(16 *t + 11.0/8) + 2.0/9 *Math.sin(17 *t + 5.0/3) + 1.0/5 *Math.sin(18 *t + 17.0/10) + 1.0/13 *Math.sin(19 *t + 19.0/8) + 23.0/24 *Math.sin(20 *t + 12.0/7) + 7.0/11 *Math.sin(23 *t + 9.0/5) + 9.0/7 *Math.sin(24 *t + 7.0/4) - 1538.0/7) *heavistep(51 *pi - t) *heavistep(t - 47 *pi) + (-2.0/7 *Math.sin(20.0/13 - 23 *t) - 1.0/6 *Math.sin(3.0/2 - 20 *t) - 5.0/7 *Math.sin(20.0/13 - 17 *t) - 1.0/9 *Math.sin(20.0/13 - 11 *t) - 1.0/6 *Math.sin(13.0/9 - 9 *t) - 19.0/6 *Math.sin(17.0/11 - 3 *t) + 263.0/5 *Math.sin(t + 11.0/7) + 614.0/15 *Math.sin(2 *t + 11.0/7) + 87.0/10 *Math.sin(4 *t + 11.0/7) + 1.0/7 *Math.sin(5 *t + 11.0/8) + 19.0/11 *Math.sin(6 *t + 11.0/7) + 7.0/5 *Math.sin(7 *t + 11.0/7) + 4.0/3 *Math.sin(8 *t + 8.0/5) + 9.0/5 *Math.sin(10 *t + 14.0/9) + 4.0/7 *Math.sin(12 *t + 8.0/5) + 3.0/11 *Math.sin(13 *t + 3.0/2) + 1.0/8 *Math.sin(14 *t + 22.0/15) + 1.0/9 *Math.sin(15 *t + 12.0/7) + 6.0/5 *Math.sin(16 *t + 11.0/7) + 2.0/9 *Math.sin(18 *t + 11.0/7) + 3.0/5 *Math.sin(19 *t + 8.0/5) + 1.0/26 *Math.sin(21 *t + 15.0/11) + 6.0/7 *Math.sin(22 *t + 8.0/5) - 1867.0/8) *heavistep(47 *pi - t) *heavistep(t - 43 *pi) + (118.0/39 *Math.sin(t + 11.0/7) + 40.0/7 *Math.sin(2 *t + 33.0/7) + 49.0/25 *Math.sin(3 *t + 14.0/3) + 12.0/5 *Math.sin(4 *t + 8.0/5) + 1.0/9 *Math.sin(5 *t + 32.0/13) + 5.0/2 *Math.sin(6 *t + 13.0/8) + 2.0/5 *Math.sin(7 *t + 22.0/5) + 3.0/4 *Math.sin(8 *t + 7.0/4) - 143.0/10) *heavistep(43 *pi - t) *heavistep(t - 39 *pi) + (-1.0/8 *Math.sin(2.0/3 - 8 *t) - 1.0/2 *Math.sin(7.0/5 - 2 *t) - 246.0/19 *Math.sin(1.0/7 - t) + 1.0/4 *Math.sin(3 *t + 33.0/16) + 1.0/6 *Math.sin(4 *t + 17.0/6) + 1.0/5 *Math.sin(5 *t + 31.0/7) + 1.0/11 *Math.sin(6 *t + 50.0/17) + 1.0/8 *Math.sin(7 *t + 30.0/7) + 665.0/6) *heavistep(39 *pi - t) *heavistep(t - 35 *pi) + (-119.0/10 *Math.sin(7.0/15 - t) + 2.0/11 *Math.sin(2 *t + 25.0/7) + 2.0/9 *Math.sin(3 *t + 5.0/8) + 1.0/5 *Math.sin(4 *t + 33.0/7) + 1.0/4 *Math.sin(5 *t + 19.0/10) + 1023.0/10) *heavistep(35 *pi - t) *heavistep(t - 31 *pi) + (-1.0/7 *Math.sin(2.0/7 - 12 *t) - 1.0/8 *Math.sin(3.0/10 - 5 *t) + 25.0/7 *Math.sin(t + 77.0/17) + 355.0/59 *Math.sin(2 *t + 41.0/40) + 27.0/5 *Math.sin(3 *t + 46.0/15) + 33.0/7 *Math.sin(4 *t + 11.0/3) + 27.0/10 *Math.sin(6 *t + 13.0/9) + 5.0/11 *Math.sin(7 *t + 11.0/5) + 5.0/8 *Math.sin(8 *t + 3) + 8.0/5 *Math.sin(9 *t + 16.0/15) + 16.0/15 *Math.sin(10 *t + 1.0/7) + 7.0/9 *Math.sin(11 *t + 12.0/5) - 862.0/7) *heavistep(31 *pi - t) *heavistep(t - 27 *pi) + (-1.0/3 *Math.sin(5.0/4 - 8 *t) - 2.0/5 *Math.sin(5.0/9 - 7 *t) - 5.0/7 *Math.sin(11.0/8 - 5 *t) - 7.0/2 *Math.sin(15.0/14 - 2 *t) + 29.0/8 *Math.sin(t + 41.0/10) + 11.0/6 *Math.sin(3 *t + 13.0/3) + 7.0/6 *Math.sin(4 *t + 1.0/27) + 2.0/7 *Math.sin(6 *t + 8.0/7) + 1.0/9 *Math.sin(9 *t + 9.0/5) + 2.0/7 *Math.sin(10 *t + 1.0/10) + 201.0/5) *heavistep(27 *pi - t) *heavistep(t - 23 *pi) + (-4.0/11 *Math.sin(8.0/9 - 12 *t) - 10.0/7 *Math.sin(19.0/13 - 10 *t) + 623.0/3 *Math.sin(t + 10.0/7) + 39.0/5 *Math.sin(2 *t + 10.0/11) + 251.0/9 *Math.sin(3 *t + 4.0/3) + 5.0/7 *Math.sin(4 *t + 4.0/3) + 61.0/6 *Math.sin(5 *t + 4.0/3) + 14.0/9 *Math.sin(6 *t + 23.0/7) + 76.0/25 *Math.sin(7 *t + 9.0/7) + 3.0/4 *Math.sin(8 *t + 1.0/4) + 19.0/5 *Math.sin(9 *t + 3.0/2) + 17.0/6 *Math.sin(11 *t + 6.0/5) + 13.0/8 *Math.sin(13 *t + 14.0/13) + 8.0/9 *Math.sin(14 *t + 17.0/6) + 24.0/25 *Math.sin(15 *t + 1.0/2) + 1.0/6 *Math.sin(16 *t + 13.0/8) + 5.0/8 *Math.sin(17 *t + 1) + 1.0/7 *Math.sin(18 *t + 18.0/17) + 6.0/7 *Math.sin(19 *t + 1) + 1.0/4 *Math.sin(20 *t + 4.0/9) + 2.0/7 *Math.sin(21 *t + 7.0/5) + 1.0/3 *Math.sin(22 *t + 8.0/7) + 2.0/5 *Math.sin(23 *t + 1.0/26) + 2.0/11 *Math.sin(24 *t + 8.0/7) - 243.0/8) *heavistep(23 *pi - t) *heavistep(t - 19 *pi) + (-111.0/10 *Math.sin(4.0/5 - 9 *t) - 12.0/5 *Math.sin(7.0/13 - 2 *t) + 1.0/6 *Math.sin(t + 48.0/11) + 13.0/8 *Math.sin(3 *t + 27.0/7) + 71.0/24 *Math.sin(4 *t + 6.0/11) + 22.0/9 *Math.sin(5 *t + 7.0/2) + 19.0/7 *Math.sin(6 *t + 8.0/17) + 20.0/7 *Math.sin(7 *t + 34.0/9) + 55.0/7 *Math.sin(8 *t + 6.0/5) + 64.0/9 *Math.sin(10 *t + 38.0/9) + 27.0/5) *heavistep(19 *pi - t) *heavistep(t - 15 *pi) + (-22.0/7 *Math.sin(4.0/3 - 8 *t) - 19.0/7 *Math.sin(20.0/13 - 6 *t) + 38.0/13 *Math.sin(t + 1.0/24) + 12.0/11 *Math.sin(2 *t + 5.0/9) + 26.0/7 *Math.sin(3 *t + 7.0/9) + 11.0/5 *Math.sin(4 *t + 12.0/11) + 37.0/10 *Math.sin(5 *t + 17.0/10) + 51.0/10 *Math.sin(7 *t + 10.0/3) + 33.0/4 *Math.sin(9 *t + 26.0/7) + 41.0/5 *Math.sin(10 *t + 9.0/5) - 27.0/2) *heavistep(15 *pi - t) *heavistep(t - 11 *pi) + (-172.0/5 *Math.sin(3.0/8 - t) + 5.0/4 *Math.sin(2 *t + 7.0/2) + 2303.0/24) *heavistep(11 *pi - t) *heavistep(t - 7 *pi) + (441.0/5 - 455.0/12 *Math.sin(7.0/9 - t)) *heavistep(7 *pi - t) *heavistep(t - 3 *pi) + (-1.0/3 *Math.sin(1.0/20 - 18 *t) - 7.0/5 *Math.sin(7.0/9 - 17 *t) - 18.0/11 *Math.sin(2.0/5 - 14 *t) - 24.0/5 *Math.sin(1.0/13 - 9 *t) + 2767.0/7 *Math.sin(t + 11.0/3) + 229.0/5 *Math.sin(2 *t + 17.0/7) + 313.0/8 *Math.sin(3 *t + 22.0/5) + 32.0/3 *Math.sin(4 *t + 22.0/5) + 169.0/6 *Math.sin(5 *t + 21.0/8) + 23.0/7 *Math.sin(6 *t + 26.0/11) + 21.0/2 *Math.sin(7 *t + 5.0/6) + 55.0/6 *Math.sin(8 *t + 14.0/5) + 212.0/13 *Math.sin(10 *t + 24.0/7) + 26.0/9 *Math.sin(11 *t + 9.0/2) + 16.0/5 *Math.sin(12 *t + 25.0/6) + 35.0/17 *Math.sin(13 *t + 4.0/11) + 15.0/8 *Math.sin(15 *t + 7.0/10) + 2.0/3 *Math.sin(16 *t + 20.0/9) + 16.0/7 *Math.sin(19 *t + 4.0/5) + 13.0/7 *Math.sin(20 *t + 29.0/7) + 14.0/3 *Math.sin(21 *t + 7.0/5) + 4.0/3 *Math.sin(22 *t + 7.0/4) + 12.0/7 *Math.sin(23 *t + 34.0/33) + 7.0/4 *Math.sin(24 *t + 27.0/7) - 211.0/5) *heavistep(3 *pi - t) *heavistep(t + pi)) *heavistep(Math.sqrt(sgn(Math.sin(t/2))))

                    ,
                    new Rotation2d(0)
            );
            target = new Pose2d(scale * target.getX(), scale * target.getY(), target.getRotation());

//            Pose2d target = new Pose2d(
//                    4 * Math.cos(runtime.seconds()),
//                    4 * Math.sin(runtime.seconds()),
//                    new Rotation2d(0)
//            );

//            Pose2d target = new Pose2d(8, 0, new Rotation2d(0));

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawTargetPose(packet.fieldOverlay(), target);
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


            double kP = 0.25;
            double[] error = {
                    target.getX() - pose.getX(),
                    target.getY() - pose.getY(),
                    normalizeAngle(target.getHeading() - pose.getHeading())
            };

            x += kP * error[0];
            y += kP * error[1];
            turn += 8 * kP * error[2];

            drive.driveFieldCentric(drive_speed * x, drive_speed * y, drive_speed * -turn, pose.getHeading());


        }
        drive.stop();

    }

    /**
     * Normalizes a given angle to [-pi,pi) degrees.
     * @param degrees the given angle in degrees.
     * @return the normalized angle in degrees.
     */
    private double normalizeAngle(double degrees) {
        double angle = degrees;
        while (opModeIsActive() && angle <= -Math.PI)
            angle += 2*Math.PI;
        while (opModeIsActive() && angle > Math.PI)
            angle -= 2*Math.PI;
        return angle;
    }

}