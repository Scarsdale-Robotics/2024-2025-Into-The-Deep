package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/**
 * Estimate robot pose with a Kalman Filter using odometry as the model, imu and camera as sensors.
 */
public class LocalizationSubsystem extends SubsystemBase {

    ///////////////////
    // KALMAN FILTER //
    ///////////////////

    // Sensor toggles.
    private boolean cameraEnabled = true;

    // Model covariance for translation
    private static final double Q_translation = 0.0625;
    // Model covariance for heading
    private static final double Q_heading = 0.0003;

    // Uncertainty
    private double P_translation;
    private double P_heading;

    // Pose
    private double x;
    private double y;
    private double h;

    // History arrays
    private final ArrayList<Double> xHistory;
    private final ArrayList<Double> yHistory;
    private final ArrayList<Double> hHistory;

    // Velocity
    private double vx;
    private double vy;
    private double vh;


    //////////////
    // ODOMETRY //
    //////////////

    // Distance between parallel odometers
    private static final double TRACK_WIDTH = 11.3386;
    // Signed distance from the point of rotation (positive=forward)
    private static final double CENTER_WHEEL_OFFSET = 0.944882;
    // Measured in inches
    private static final double WHEEL_DIAMETER = 1.37795;
    // Odometer encoder resolution
    private static final double TICKS_PER_REV = 4096;
    // Circumference divided by encoder resolution [inches/tick]
    private static final double DISTANCE_PER_PULSE = (Math.PI * WHEEL_DIAMETER) / TICKS_PER_REV;

    // Inbuilt odometry object
    private final Encoder leftOdometer;
    private final Encoder rightOdometer;
    private final Encoder centerOdometer;
    private final HolonomicOdometry odometry;
    private Pose2d lastOdometryPose;


    ////////////
    // CAMERA //
    ////////////
    private final CVSubsystem cv;


    //////////
    // MISC //
    //////////
    private final ArrayList<Double> dtHistory;
    private final ElapsedTime runtime;
    private double lastTime;
    private double deltaTime;
    private double averageDeltaTime;

    private Telemetry telemetry = null;

    /**
     * Creates a new LocalizationSubsystem object with the given parameters.
     * @param initialPose The robot's starting pose.
     * @param leftOdometer Encoder object.
     * @param rightOdometer Encoder object.
     * @param centerOdometer Encoder object.
     * @param cv The CVSubsystem of the robot.
     */
    public LocalizationSubsystem(
            Pose2d initialPose,
            Encoder leftOdometer,
            Encoder rightOdometer,
            Encoder centerOdometer,
            CVSubsystem cv) {

        // Init KF
        this.P_translation = 2;
        this.P_heading = 0.1;
        this.x = initialPose.getX();
        this.y = initialPose.getY();
        this.h = initialPose.getHeading();
        this.xHistory = new ArrayList<>();
        this.xHistory.add(x);
        this.yHistory = new ArrayList<>();
        this.yHistory.add(y);
        this.hHistory = new ArrayList<>();
        this.hHistory.add(h);
        this.vx = 0;
        this.vy = 0;
        this.vh = 0;


        // Init odometry
        // Encoders
        this.leftOdometer = leftOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        this.rightOdometer = rightOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        this.centerOdometer = centerOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);

        this.leftOdometer.reset();
        this.rightOdometer.reset();
        this.centerOdometer.reset();
        this.odometry = new HolonomicOdometry(
                this.leftOdometer::getDistance,
                this.rightOdometer::getDistance,
                this.centerOdometer::getDistance,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );
        this.lastOdometryPose = new Pose2d(0,0,new Rotation2d(0));


        // Store camera
        this.cv = cv;
        enableCamera();


        // Init time
        this.runtime = new ElapsedTime();
        this.runtime.reset();
        this.lastTime = -1;
        this.deltaTime = 1;
        this.dtHistory = new ArrayList<>();
        this.dtHistory.add(1d);
        this.averageDeltaTime = 1;

        // Init telemetry
        this.telemetry = null;

    }

    /**
     * Creates a new LocalizationSubsystem object with the given parameters.
     * @param initialPose The robot's starting pose.
     * @param leftOdometer Encoder object.
     * @param rightOdometer Encoder object.
     * @param centerOdometer Encoder object.
     * @param cv The CVSubsystem of the robot.
     * @param telemetry The opmode's Telemetry object.
     */
    public LocalizationSubsystem(
            Pose2d initialPose,
            Encoder leftOdometer,
            Encoder rightOdometer,
            Encoder centerOdometer,
            CVSubsystem cv,
            Telemetry telemetry) {
        this(
                initialPose,
                leftOdometer,
                rightOdometer,
                centerOdometer,
                cv
        );
        this.telemetry = telemetry;
    }

    /**
     * Creates a new LocalizationSubsystem object with the given parameters.
     * @param initialPose The robot's starting pose.
     * @param leftOdometer Encoder object.
     * @param rightOdometer Encoder object.
     * @param centerOdometer Encoder object.
     * @param telemetry The opmode's Telemetry object.
     */
    public LocalizationSubsystem(
            Pose2d initialPose,
            Encoder leftOdometer,
            Encoder rightOdometer,
            Encoder centerOdometer,
            Telemetry telemetry) {
        this(
                initialPose,
                leftOdometer,
                rightOdometer,
                centerOdometer,
                (CVSubsystem) null
        );
        this.telemetry = telemetry;
        disableCamera();
    }


    ////////////////////
    // SENSOR TOGGLES //
    ////////////////////

    /**
     * Disables the camera/AprilTag sensor.
     */
    public void disableCamera() {
        cameraEnabled = false;
    }

    /**
     * Enables the camera/AprilTag sensor.
     */
    public void enableCamera() {
        cameraEnabled = true;
    }



    /////////////
    // GETTERS //
    /////////////

    /**
     * @return The current pose as a Pose2d object.
     */
    public Pose2d getPose() {
        return new Pose2d(x, y, new Rotation2d(h));
    }

    /**
     * @return The current X position in inches.
     */
    public double getX() {
        return x;
    }

    /**
     * @return The current Y position in inches.
     */
    public double getY() {
        return y;
    }

    /**
     * @return The current heading in radians.
     */
    public double getH() {
        return h;
    }

    /**
     * @return The current velocity as a Pose2d object.
     */
    public Pose2d getVelocity() {
        return new Pose2d(vx, vy, new Rotation2d(vh));
    }

    /**
     * @return The current X velocity in inches/second.
     */
    public double getVX() {
        return vx;
    }

    /**
     * @return The current Y velocity in inches/second.
     */
    public double getVY() {
        return vy;
    }

    /**
     * @return The current heading velocity in radians/second.
     */
    public double getVH() {
        return vh;
    }



    //////////////////////////
    // LOCALIZATION METHODS //
    //////////////////////////

    /**
     * Prediction step of the Kalman Filter.
     */
    private void predictKF() {
        // Get odometry pose
        odometry.updatePose();
        Pose2d currentPose = odometry.getPose();
        currentPose = new Pose2d(currentPose.getX(), -currentPose.getY(), new Rotation2d(-currentPose.getHeading()));

        // Calculate model
        // rotate odometry displacement by h-lastOdometryPose.getHeading()
        double odom_dx = currentPose.getX() - lastOdometryPose.getX();
        double odom_dy = currentPose.getY() - lastOdometryPose.getY();
        double beta = normalizeAngle(h - lastOdometryPose.getHeading());
        telemetry.addData("_DX", odom_dx);
        telemetry.addData("_DY", odom_dy);
        telemetry.addData("_BETA", beta);
        double dx = odom_dx*Math.cos(beta) - odom_dy*Math.sin(beta);
        double dy = odom_dx*Math.sin(beta) + odom_dy*Math.cos(beta);
        x += dx;
        y += dy;

        double odom_dh = normalizeAngle(currentPose.getHeading() - lastOdometryPose.getHeading());
        telemetry.addData("_DH", odom_dh);
        h = normalizeAngle(h + odom_dh);
        telemetry.addData("_PREDICT_H", h);

        lastOdometryPose = currentPose;


        telemetry.update();

        // Add uncertainty
        // TODO: TUNE
        P_translation += Math.hypot(dx, dy) * Q_translation;
        P_heading += Math.abs(odom_dh) * Q_heading;

        // Telemetry
        if (telemetry!=null) {
            telemetry.addData("Odom X", x);
            telemetry.addData("Odom Y", y);
            telemetry.addData("Odom H", Math.toDegrees(h)+"°");
            telemetry.addData("P_translation", P_translation);
            telemetry.addData("P_heading", P_heading);
        }
    }

    /**
     * Update step for the heading Kalman Filter.
     */
    private void updateHeadingKF() {
        CVSubsystem.PoseEstimation cameraEstimation = null;
        if (cameraEnabled) cameraEstimation = cv.getPoseEstimation();
        if (cameraEstimation ==null) return;

        Pose2d cameraPose = cameraEstimation.pose;
        telemetry.addData("heading covariance original", cameraEstimation.headingCovariance);
        telemetry.addData("heading covariance increase", Math.pow(normalizeAngle(cameraPose.getHeading() - h), 2));
        telemetry.update();
        cameraEstimation.headingCovariance += Math.pow(normalizeAngle(cameraPose.getHeading() - h), 2);

        // Update heading
        double R_camera_heading = cameraEstimation.headingCovariance;
        double K_heading = P_heading / (P_heading + R_camera_heading);
        h += K_heading * (normalizeAngle(cameraPose.getHeading() - h));
        h = normalizeAngle(h);
        P_heading = (1 - K_heading) * P_heading;

        // Telemetry
        if (telemetry!=null) {
            telemetry.addData("Apriltag H", Math.toDegrees(cameraPose.getHeading()) + "°");
        }

        // Update MT2 heading.
        cv.updateHeading(h);
    }

    /**
     * Update step for the translation Kalman Filter.
     */
    private void updateTranslationKF() {
        CVSubsystem.PoseEstimation cameraEstimation = null;
        if (cameraEnabled) cameraEstimation = cv.getPoseEstimation();
        if (cameraEstimation ==null) return;

        Pose2d cameraPose = cameraEstimation.pose;
        cameraEstimation.translationCovariance += Math.pow(Math.hypot(cameraPose.getX() - x, cameraPose.getY() - y), 2);

        // Update translation
        double R_camera_translation = cameraEstimation.translationCovariance;
        double K_translation = P_translation / (P_translation + R_camera_translation);
        x += K_translation * (cameraPose.getX() - x);
        y += K_translation * (cameraPose.getY() - y);
        P_translation = (1 - K_translation) * P_translation;

        // Telemetry
        if (telemetry!=null) {
            telemetry.addData("AprilTag X", cameraPose.getX());
            telemetry.addData("Apriltag Y", cameraPose.getY());
        }
    }

    /**
     * @param a The process value array.
     * @return Approximated derivative according to the Five-Point stencil.
     */
    public double stencil(ArrayList<Double> a) {
        return (-a.get(4) + 8*a.get(3) - 8*a.get(1) + a.get(0)) /
                (12 * averageDeltaTime);
    }

    /**
     * Calculates the translational and rotational velocities using the Five-Point stencil.
     */
    private void updateVelocity() {
        // Calculate deltaTime
        double currentTime = runtime.seconds();
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        // Update history arrays
        dtHistory.add(deltaTime);
        xHistory.add(x);
        yHistory.add(y);
        hHistory.add(h);
        while (dtHistory.size()>5) dtHistory.remove(0);
        while (xHistory.size()>5) xHistory.remove(0);
        while (yHistory.size()>5) yHistory.remove(0);
        while (hHistory.size()>5) hHistory.remove(0);

        // Find derivatives
        averageDeltaTime = dtHistory.stream().mapToDouble(aa -> aa).average().orElse(0);
        if (xHistory.size()==5) vx = stencil(xHistory);
        if (yHistory.size()==5) vy = stencil(yHistory);
        if (hHistory.size()==5) vh = stencil(hHistory);
    }

    /**
     * Update the pose and velocity.
     */
    public void update() {

        // Kalman Filter steps
        predictKF();
        updateHeadingKF();
        updateTranslationKF();

        // Calculate velocity
        updateVelocity();

        // Telemetry
        if (telemetry!=null) {
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("H", Math.toDegrees(h)+"°");
            telemetry.addData("VX", vx+"in/s");
            telemetry.addData("VY", vy+"in/s");
            telemetry.addData("VH", Math.toDegrees(vh)+"°/s");
            telemetry.update();
        }
    }

    /**
     * Normalizes a given angle to (-pi,pi] radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2*Math.PI;
        while (radians <= -Math.PI) radians += 2*Math.PI;
        return radians;
    }

}