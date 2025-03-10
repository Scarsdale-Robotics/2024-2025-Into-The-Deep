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

    private static final double xOffset = 92.0, yOffset = 48.0;
    private static final double ENCODER_CPR = 4096; // Optii v1
    private static final double ODOM_DIAMETER = 35.0; // mm
    private static final double TICKS_PER_MM = ENCODER_CPR / (Math.PI * ODOM_DIAMETER);

    public GoBildaPinpointDriver pinpoint;

    private Pose2d lastOdometryPose;


    //////////
    // MISC //
    //////////
    private final ArrayList<Double> dtHistory;
    private final ElapsedTime runtime;
    private double lastTime;
    private double deltaTime;
    private double averageDeltaTime;

    private Telemetry telemetry;






    /**
     * Creates a new LocalizationSubsystem object with the given parameters.
     * @param initialPose The robot's starting pose.
     * @param telemetry The opmode's Telemetry object.
     */
    public LocalizationSubsystem(
            Pose2d initialPose,
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


        // Init time
        this.runtime = new ElapsedTime();
        this.runtime.reset();
        this.lastTime = -1;
        this.deltaTime = 1;
        this.dtHistory = new ArrayList<>();
        this.dtHistory.add(1d);
        this.averageDeltaTime = 1;


        // Init pinpoint
        this.pinpoint = pinpoint;
        this.pinpoint.setOffsets(xOffset, yOffset);
        this.pinpoint.setEncoderResolution(TICKS_PER_MM);

        // wait for pinpoint to be ready
        this.telemetry.addData("[LOCALIZATION]", "initializing pinpoint");
        this.telemetry.update();
        while ((opMode.opModeIsActive() || opMode.opModeInInit())
                && this.pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            this.pinpoint.update();
            this.telemetry.addData("[PP STATUS]", this.pinpoint.getDeviceStatus());
            this.telemetry.update();
        }
        this.telemetry.addData("[PP STATUS]", this.pinpoint.getDeviceStatus());
        this.telemetry.update();
        this.pinpoint.update();

        // set pinpoint initial position
        this.telemetry.addData("[LOCALIZATION]", "setting pinpoint initial position");
        this.telemetry.update();
        Pose2D initialPose2D = new Pose2D(DistanceUnit.INCH, initialPose.getX(), initialPose.getY(), AngleUnit.RADIANS, initialPose.getHeading());
        double initialX = initialPose.getX();
        double initialY = initialPose.getY();
        double initialH = initialPose.getHeading();
        Pose2D ppPose = this.pinpoint.getPosition();
        double ppX = ppPose.getX(DistanceUnit.INCH);
        double ppY = ppPose.getY(DistanceUnit.INCH);
        double ppH = ppPose.getHeading(AngleUnit.RADIANS);
        while ((opMode.opModeIsActive() || opMode.opModeInInit())
                && !(equal(ppX,initialX) && equal(ppY,initialY) && equal(ppH,initialH))) {
            this.pinpoint.setPosition(initialPose2D);
            this.pinpoint.update();
            ppPose = this.pinpoint.getPosition();
            ppX = ppPose.getX(DistanceUnit.INCH);
            ppY = ppPose.getY(DistanceUnit.INCH);
            ppH = ppPose.getHeading(AngleUnit.RADIANS);

            // Telemetry
            this.telemetry.addData("[LOCALIZATION] ppX", ppX);
            this.telemetry.addData("[LOCALIZATION] ppY", ppY);
            this.telemetry.addData("[LOCALIZATION] ppH", ppH);
            this.telemetry.addData("[LOCALIZATION] initialH", initialH);
            this.telemetry.update();
        }
        this.telemetry.addData("[L. SUB STATUS]", "finished initializing pinpoint");
        this.telemetry.update();
        this.lastOdometryPose = initialPose;

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
        // no update step, we're not using apriltags

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

    /**
     * Determines whether the two inputs are approximately equal to each other
     * within an epsilon of 1e-3
     * @param a
     * @param b
     * @return Math.abs(a-b) <= 1e-3
     */
    private static boolean equal(double a, double b) {
        return Math.abs(a-b) <= 1e-3;
    }

}