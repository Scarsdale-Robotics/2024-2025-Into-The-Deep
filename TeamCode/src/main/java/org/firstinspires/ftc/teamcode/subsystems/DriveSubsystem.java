package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive controller;
    private final AdafruitBNO055IMU imu;

    private final LinearOpMode opMode;

    private final Motor leftFront;
    private final Motor rightFront;
    private final Motor leftBack;
    private final Motor rightBack;

    private Telemetry telemetry;

    private double imuResetValue;

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, AdafruitBNO055IMU imu, LinearOpMode opMode, Telemetry telemetry) {
        this.rightBack = rightBack;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.leftFront = leftFront;
        controller = new MecanumDrive(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );
        this.imu = imu;
        this.opMode = opMode;
        this.telemetry = telemetry;

        if (this.telemetry != null) {
            this.imuResetValue = getYaw();
            this.telemetry.addData("HEADING", getYaw());
            this.telemetry.update();
        }
    }

    /**
     * Drives with directions based on robot pov.
     *
     * @param right     How much right the robot should strafe (negative values = strafe left).
     * @param forward   How much forward the robot should move (negative values = move backwards).
     * @param turn      How much the robot should turn.
     */
    public void driveRobotCentric(double right, double forward, double turn) {
        controller.driveRobotCentric(right, forward, turn);
    }

    /**
     * Drives with directions based on driver pov.
     *
     * @param right     How much right the robot should strafe (negative values = strafe left).
     * @param forward   How much forward the robot should move (negative values = move backwards).
     * @param turn      How much the robot should turn.
     */
    public void driveFieldCentric(double right, double forward, double turn) {
        double yaw = getYaw();
        controller.driveFieldCentric(right, forward, turn, yaw);
    }



    /**
     * @return the current position of the robot's back left wheel in ticks.
     */
    public int getLBPosition() {
        return leftBack.getCurrentPosition();
    }

    /**
     * @return the current position of the robot's back right wheel in ticks.
     */
    public int getRBPosition() {
        return rightBack.getCurrentPosition();
    }

    /**
     * @return the current position of the robot's front left wheel in ticks.
     */
    public int getLFPosition() {
        return leftFront.getCurrentPosition();
    }

    /**
     * @return the current position of the robot's front right wheel in ticks.
     */
    public int getRFPosition() {
        return rightFront.getCurrentPosition();
    }

    /**
     * Stop the motors.
     */
    public void stopController() {
        controller.stop();
    }

    public AdafruitBNO055IMU getIMU() {
        return imu;
    }

    public void resetIMU() {
        imu.resetDeviceConfigurationForOpMode();
        imu.initialize(new BNO055IMU.Parameters());
        imuResetValue = imu.getAngularOrientation().firstAngle * 180.0 / Math.PI; // degrees
    }

    public double getYaw() {
        return normalizeAngle(imu.getAngularOrientation().firstAngle * 180.0 / Math.PI - imuResetValue);
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