package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

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


    /**
     * Constructs an OdometrySubsystem with a specific odometry implementation.
     *
     * @param odometry the odometry mechanism on the robot.
     */
    public OdometrySubsystem(HardwareMap hardwareMap) {

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

    }

    /**
     * Returns the current pose of the robot.
     *
     * @return The current pose as a Pose2d object.
     */
    public Pose2d getPose() {
        // Return the current pose from the odometry system.
        return odometry.getPose();
    }

    /**
     * Updates the robot's pose using the odometry system.
     * This should be called at the end of every loop to keep the pose estimate up to date.
     */
    public void update() {
        // Update the pose estimate through the odometry system.
        odometry.updatePose();
    }

    /**
     * Automatically updates the pose every cycle.
     */
    @Override
    public void periodic() {
        // Keep the odometry system's pose estimate up to date each cycle.
        update();
    }

}