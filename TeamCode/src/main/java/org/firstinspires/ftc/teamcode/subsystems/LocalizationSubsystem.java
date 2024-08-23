package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Estimate robot pose with a Kalman Filter using odometry as the model, imu and camera as sensors.
 */
public class LocalizationSubsystem extends SubsystemBase {

    ///////////////////
    // KALMAN FILTER //
    ///////////////////

    // Model covariance for translation
    private static final double Q_translation = 0.62;
    // Model covariance for heading
    private static final double Q_heading = 1;
    // Sensor (Camera) covariance for translation
    private static final double R_camera_translation = 0.03875;
    // Sensor (Camera) covariance for heading
    private static final double R_camera_heading = 0.04;
    // Sensor (IMU) covariance for heading
    private static final double R_imu_heading = 0.25;


    // Uncertainty
    private double P_translation;
    private double P_heading;

    // Pose
    private double x;
    private double y;
    private double h;

    // Velocity
    private double vx;
    private double vy;
    private double vh;


    //////////////////////
    // ODOMETRY (MODEL) //
    //////////////////////

    // Distance between parallel odometers
    private static final double TRACK_WIDTH = 12.4; //TODO: tune
    // Signed distance from the point of rotation (positive=forward)
    private static final double CENTER_WHEEL_OFFSET = 0; //TODO: tune
    // Measured in inches
    private static final double WHEEL_DIAMETER = 1.37795;
    // Odometer encoder resolution
    private static final double TICKS_PER_REV = 4096;
    // Circumference divided by encoder resolution [inches/tick]
    private static final double DISTANCE_PER_PULSE = (Math.PI * WHEEL_DIAMETER) / TICKS_PER_REV;

    // Encoders
    private final Encoder leftOdometer;
    private final Encoder rightOdometer;
    private final Encoder centerOdometer;

    // Inbuilt odometry object
    private final HolonomicOdometry odometry;


    //////////////////
    // IMU (SENSOR) //
    //////////////////
    private final AdafruitBNO055IMU imu;
    private double imuBias;


    //////////
    // MISC //
    //////////
    private final ElapsedTime runtime;
    private double lastTime;
    private Telemetry telemetry = null;

    /**
     * Creates a new LocalizationSubsystem object with the given parameters.
     * @param initialPose The robot's starting pose.
     * @param leftOdometer Encoder object.
     * @param rightOdometer Encoder object.
     * @param centerOdometer Encoder object.
     * @param imu AdafruitBNO055 object.
     */
    public LocalizationSubsystem(
            Pose2d initialPose,
            Encoder leftOdometer,
            Encoder rightOdometer,
            Encoder centerOdometer,
            AdafruitBNO055IMU imu) {

        // Init KF
        this.P_translation = 0;
        this.P_heading = 0;
        this.x = initialPose.getX();
        this.y = initialPose.getY();
        this.h = initialPose.getHeading();
        this.vx = 0;
        this.vy = 0;
        this.vh = 0;


        // Init odometry
        this.leftOdometer = leftOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        this.rightOdometer = rightOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        this.centerOdometer = centerOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);

        this.leftOdometer.reset();
        this.rightOdometer.reset();
        this.centerOdometer.reset();

        this.rightOdometer.setDirection(Motor.Direction.REVERSE);
        this.odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );
        this.odometry.updatePose(initialPose);


        // Init IMU
        this.imu = imu;
        this.imuBias = 0;
        updateYaw(this.h);


        // Init time
        this.runtime = new ElapsedTime();
        this.runtime.reset();
        this.lastTime = -1;

    }

    /**
     * Creates a new LocalizationSubsystem object with the given parameters.
     * @param initialPose The robot's starting pose.
     * @param leftOdometer Encoder object.
     * @param rightOdometer Encoder object.
     * @param centerOdometer Encoder object.
     * @param imu AdafruitBNO055 object.
     * @param telemetry The opmode's Telemetry object.
     */
    public LocalizationSubsystem(
            Pose2d initialPose,
            Encoder leftOdometer,
            Encoder rightOdometer,
            Encoder centerOdometer,
            AdafruitBNO055IMU imu,
            Telemetry telemetry) {
        this(
                initialPose,
                leftOdometer,
                rightOdometer,
                centerOdometer,
                imu
        );
        this.telemetry = telemetry;
    }


    /**
     * @return The current pose as a Pose2d object.
     */
    public Pose2d getPose() {
        return new Pose2d(x, y, new Rotation2d(h));
    }

    /**
     * @return The current velocity as a Pose2d object.
     */
    public Pose2d getVelocity() {
        return new Pose2d(vx, vy, new Rotation2d(vh));
    }

    /**
     * Prediction step of the Kalman Filter.
     */
    private void predict() {
        // Calculate model
        odometry.updatePose();
        Pose2d currentPose = odometry.getPose();
        x = currentPose.getX();
        y = currentPose.getY();
        h = currentPose.getHeading();

        // Add uncertainty
        P_translation += Q_translation;
        P_heading += Q_heading;

        // Telemetry
        if (telemetry!=null) {
            telemetry.addData("Odom X", x);
            telemetry.addData("Odom Y", y);
            telemetry.addData("Odom H", Math.toDegrees(h));
        }
    }

    /**
     * Update step of the Kalman Filter.
     */
    private void update() {

        // TODO: ADD CAMERA CODE
        if ([insert whether or not apriltag is found]) {
            // Telemetry
            if (telemetry!=null) {
                telemetry.addData("AprilTag X",[insert apriltag x]);
                telemetry.addData("Apriltag Y",[insert apriltag y]);
                telemetry.addData("Apriltag H",Math.toDegrees([insert apriltag h]));
                telemetry.addData("IMU H", Math.toDegrees(getYaw()));
            }

            // Update translation
            double K_translation = P_translation / (P_translation + R_camera_translation);
            x += K_translation * ([apriltag x] - x);
            y += K_translation * ([apriltag y] - y);
            P_translation = (1 - K_translation) * P_translation;

            // Update heading
            double R_combined_heading = (R_imu_heading * R_camera_heading) / (R_imu_heading + R_camera_heading);
            double K_combined_heading = P_heading / (P_heading + R_combined_heading);
            double imu_difference_heading = normalizeAngle(getYaw() - h);
            double camera_difference_heading = normalizeAngle([apriltag h] - h);
            double combined_residual_heading =
                    R_camera_heading / (R_imu_heading + R_camera_heading) * imu_difference_heading +
                    R_imu_heading / (R_imu_heading + R_camera_heading) * camera_difference_heading;
            h += K_combined_heading * combined_residual_heading;
            h = normalizeAngle(h);
            P_heading = (1 - K_combined_heading) * P_heading;
        }
        else {
            // Telemetry
            if (telemetry!=null) {
                telemetry.addData("IMU H", Math.toDegrees(getYaw()));
            }

            // Update heading
            double K_heading = P_heading / (P_heading + R_imu_heading);
            h += K_heading * (normalizeAngle(getYaw() - h));
            h = normalizeAngle(h);
            P_heading = (1 - K_heading) * P_heading;
        }
    }

    /**
     * Corrects the pose of the odometry and yaw of the IMU with the new state estimation.
     */
    private void correct() {
        odometry.updatePose(new Pose2d(x, y, new Rotation2d(h)));
        updateYaw(h);

        // Telemetry
        if (telemetry!=null) {
            telemetry.addData("Filtered X", x);
            telemetry.addData("Filtered Y", y);
            telemetry.addData("Filtered H", Math.toDegrees(h));
        }
    }

    /**
     * Update the pose and velocity.
     */
    public void updateState() {
        // Telemetry
        if (telemetry!=null) telemetry.clearAll();

        // Cache previous pose estimate
        double x_last = x;
        double y_last = y;
        double h_last = h;

        // Kalman Filter steps
        predict();
        update();
        correct();

        // Calculate velocity
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        if (deltaTime>0) {
            vx = (x - x_last) / deltaTime;
            vy = (y - y_last) / deltaTime;
            vh = (h - h_last) / deltaTime;
        }
        else {
            vx = 0;
            vy = 0;
            vh = 0;
        }

        // Telemetry
        if (telemetry!=null) {
            telemetry.addData("VX", vx);
            telemetry.addData("VY", vy);
            telemetry.addData("VH", Math.toDegrees(vh));
            telemetry.update();
        }
    }

    /**
     * Reset the IMU to the given yaw in radians.
     * @param yaw The robot's yaw in radians.
     */
    public void updateYaw(double yaw) {
        imuBias = normalizeAngle(yaw - imu.getAngularOrientation().firstAngle);
    }

    /**
     * @return the IMU yaw in radians.
     */
    public double getYaw() {
        return normalizeAngle(imu.getAngularOrientation().firstAngle + imuBias);
    }

    /**
     * Normalizes a given angle to [-pi,pi) radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private double normalizeAngle(double radians) {
        return (radians - Math.PI) % (2*Math.PI) + Math.PI;
    }

}