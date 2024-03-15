package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OdometrySubsystem extends SubsystemBase {

    protected HolonomicOdometry odometry;

    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACK_WIDTH = 14.7; //TODO: tune

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = -2.1; //TODO: tune

    public static final double WHEEL_DIAMETER = 2.0; //TODO: tune/grab from gobilda documentation
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192; //TODO: tune/grab from gobilda documentation
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private Encoder leftOdometer, rightOdometer, centerOdometer;

    private Pose2d lastPose, currentPose, currentVelocity, deltaPose;
    private double lastTime, currentTime, deltaTime;
    private final ElapsedTime runtime;

    private final LinearOpMode opMode;


    /**
     * Constructs an OdometrySubsystem using components derived from a HardwareMap.
     *
     * @param hardwareMap the HardwareMap for the robot.
     */
    public OdometrySubsystem(HardwareMap hardwareMap, LinearOpMode opMode) {

        MotorEx frontLeft, frontRight, backLeft;

        frontLeft = new MotorEx(hardwareMap, "front_left");
        frontRight = new MotorEx(hardwareMap, "front_right");
        backLeft = new MotorEx(hardwareMap, "back_left");

        leftOdometer = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightOdometer.setDirection(Motor.Direction.REVERSE);

        this.odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        this.opMode = opMode;

        // init pose and velocity tracking
        this.currentPose = this.odometry.getPose();
        this.lastPose = this.currentPose;
        this.currentVelocity = new Pose2d(0,0,new Rotation2d(0));

        // init time tracking
        this.runtime = new ElapsedTime();
        this.runtime.reset();
        this.lastTime = runtime.seconds();
        this.currentTime = lastTime;

    }

    /**
     * Returns the current pose of the robot.
     *
     * @return The current pose as a Pose2d object.
     */
    public Pose2d getPose() {
        return currentPose;
    }

    /**
     * Returns the current velocity of the robot.
     *
     * @return The current velocity as a Pose2d object.
     */
    public Pose2d getVelocity() {
        return currentVelocity;
    }

    /**
     * Updates the robot's pose using the odometry system.
     * This should be called at the end of every loop to keep the pose estimate up to date.
     */
    public void update() {

        // get current and delta time
        currentTime = runtime.seconds();
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        // get current and delta pose
        odometry.updatePose();
        currentPose = odometry.getPose();
        deltaPose = new Pose2d(
                currentPose.getX()-lastPose.getX(),
                currentPose.getY()-lastPose.getY(),
                new Rotation2d(normalizeAngle(currentPose.getHeading()-lastPose.getHeading()))
        );
        lastPose = currentPose;

        if (Math.abs(deltaTime) > 1e-6) {
            // calculate velocities
            currentVelocity = new Pose2d(
                    deltaPose.getX()/deltaTime,
                    deltaPose.getY()/deltaTime,
                    new Rotation2d(deltaPose.getHeading()/deltaTime)
            );
        }

    }

    /**
     * Automatically updates the pose every cycle.
     */
    @Override
    public void periodic() {
        // Keep the odometry system's pose estimate up to date each cycle.
        update();
    }

    /**
     * Normalizes a given angle to [-180,180) degrees.
     * @param degrees the given angle in degrees.
     * @return the normalized angle in degrees.
     */
    private double normalizeAngle(double degrees) {
        double angle = degrees;
        while (opMode.opModeIsActive() && angle <= -180)
            angle += 360;
        while (opMode.opModeIsActive() && angle > 180)
            angle -= 360;
        return angle;
    }

}