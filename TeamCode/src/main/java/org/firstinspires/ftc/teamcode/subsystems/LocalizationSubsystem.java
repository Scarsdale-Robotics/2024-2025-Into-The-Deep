package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;

/**
 * Estimate robot pose with a Kalman Filter using odometry as the model, imu and camera as sensors.
 */
@Config
public class LocalizationSubsystem extends SubsystemBase {

    ///////////////////
    // KALMAN FILTER //
    ///////////////////

    // Sensor toggles.
    private boolean cameraEnabled = true;

    // Model covariance for translation
    private static final double Q_translation = 0.01849838438;
    // Model covariance for heading
    private static final double Q_heading = 0.0001;

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
    // PINPOINT //
    //////////////

    private static final double xOffset = 144.0, yOffset = -120.0;
    private static final double ENCODER_CPR = 4096; // Optii v1
    private static final double ODOM_DIAMETER = 35.0; // mm
    private static final double TICKS_PER_MM = ENCODER_CPR / (Math.PI * ODOM_DIAMETER);

    public GoBildaPinpointDriver pinpoint;

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
     * @param cv The CVSubsystem of the robot.
     * @param telemetry The opmode's Telemetry object.
     */
    public LocalizationSubsystem(
            Pose2d initialPose,
            CVSubsystem cv,
            GoBildaPinpointDriver pinpoint,
            LinearOpMode opMode,
            Telemetry telemetry) {

        // Init telemetry
        this.telemetry = telemetry;

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


        // Init pinpoint
        this.pinpoint = pinpoint;
        this.pinpoint.setOffsets(xOffset, yOffset);
        this.pinpoint.setEncoderResolution(TICKS_PER_MM);
        this.pinpoint.resetPosAndIMU();
        while (this.pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY && opMode.opModeInInit()) {
            this.pinpoint.update();
            this.telemetry.addData("[L. SUB STATUS]", "init pinpoint");
            this.telemetry.addData("[PP STATUS]", this.pinpoint.getDeviceStatus());
            this.telemetry.update();
        }
        this.telemetry.addData("[L. SUB STATUS]", "finished pp");
        Pose2D initialPose2D = new Pose2D(DistanceUnit.INCH, initialPose.getX(), initialPose.getY(), AngleUnit.RADIANS, initialPose.getHeading());
        this.pinpoint.update();
        this.pinpoint.setPosition(initialPose2D);
        this.pinpoint.update();
        this.telemetry.addData("initialPose2D.getH(AngleUnit.RADIANS)", initialPose2D.getHeading(AngleUnit.RADIANS));
        this.telemetry.addData("this.pinpoint.getPosX()", this.pinpoint.getPosX());
        this.telemetry.addData("this.pinpoint.getPosY()", this.pinpoint.getPosY());
        this.telemetry.addData("this.pinpoint.getHeading()", this.pinpoint.getHeading());
        this.telemetry.update();
        this.lastOdometryPose = initialPose;


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
     * Reset the heading to the given value.
     * @param H
     */
    public void resetH(double H) {
        h = H;
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
        pinpoint.update();
        Pose2D pinpointPose = pinpoint.getPosition();
        Pose2d currentPose = new Pose2d(pinpointPose.getX(DistanceUnit.INCH), pinpointPose.getY(DistanceUnit.INCH), new Rotation2d(pinpointPose.getHeading(AngleUnit.RADIANS)));

        // Calculate model
        // rotate odometry displacement by h-lastOdometryPose.getHeading()
        double odom_dx = currentPose.getX() - lastOdometryPose.getX();
        double odom_dy = currentPose.getY() - lastOdometryPose.getY();
        double beta = normalizeAngle(h - lastOdometryPose.getHeading());
        double dx = odom_dx*Math.cos(beta) - odom_dy*Math.sin(beta);
        double dy = odom_dx*Math.sin(beta) + odom_dy*Math.cos(beta);
        x += dx;
        y += dy;

        double odom_dh = normalizeAngle(currentPose.getHeading() - lastOdometryPose.getHeading());
        h = normalizeAngle(h + odom_dh);

        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(h)) {
            x = lastOdometryPose.getX();
            y = lastOdometryPose.getY();
            h = lastOdometryPose.getHeading();
        } else {
            lastOdometryPose = currentPose;
        }


        // Add uncertainty
        P_translation += Math.hypot(dx, dy) * Q_translation;
        P_heading += Math.abs(odom_dh) * Q_heading;

        // Telemetry
        if (telemetry!=null) {
            telemetry.addData("Odom X", x);
            telemetry.addData("Odom Y", y);
            telemetry.addData("Odom H", Math.toDegrees(h)+"°");
            telemetry.addData("P_translation", P_translation);
            telemetry.addData("P_heading", P_heading);
            telemetry.addData("PP X", pinpointPose.getX(DistanceUnit.INCH));
            telemetry.addData("PP Y", pinpointPose.getY(DistanceUnit.INCH));
            telemetry.addData("PP H", pinpointPose.getHeading(AngleUnit.DEGREES));
        }
    }

    /**
     * Update step for the heading Kalman Filter.
     */
    private void updateHeadingKF(CVSubsystem.PoseEstimation cameraEstimation) {
        Pose2d cameraPose = cameraEstimation.pose;
        cameraEstimation.headingCovariance += 0.25*Math.pow(normalizeAngle(cameraPose.getHeading() - h), 2);

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
    private void updateTranslationKF(CVSubsystem.PoseEstimation cameraEstimation) {
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
        // Get AprilTag result
        CVSubsystem.PoseEstimation cameraEstimation = null;
        if (cameraEnabled) cameraEstimation = cv.getPoseEstimation();

        // Kalman Filter steps
        predictKF();
        if (cameraEstimation != null) {
            updateHeadingKF(cameraEstimation);
            updateTranslationKF(cameraEstimation);
        }

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