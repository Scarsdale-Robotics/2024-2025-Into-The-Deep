package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.synchropather.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    /// MOTORS ///
    private final Motor leftFront;
    private final Motor rightFront;
    private final Motor leftBack;
    private final Motor rightBack;
    private final MecanumDrive controller;

    /// POWERS ///
    public double driveSpeed;
    public double driveTheta;
    public double turnVelocity;

    public DriveSubsystem(Motor leftFront,
                          Motor rightFront,
                          Motor leftBack,
                          Motor rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.controller = new MecanumDrive(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );
        this.driveSpeed = 0;
        this.driveTheta = 0;
        this.turnVelocity = 0;
    }



    /////////////
    // GETTERS //
    /////////////

    public double getLeftFrontPosition() {
        return -leftFront.getCurrentPosition();
    }

    public double getRightFrontPosition() {
        return -rightFront.getCurrentPosition();
    }

    public double getLeftBackPosition() {
        return leftBack.getCurrentPosition();
    }

    public double getRightBackPosition() {
        return -rightBack.getCurrentPosition();
    }



    ///////////////////
    // DRIVE METHODS //
    ///////////////////

    /**
     * Drives with directions based on robot pov.
     *
     * @param theta     Direction of drive in radians.
     * @param speed     Desired driving speed in in/s.
     * @param turn      Desired angular velocity in rad/s.
     */
    public void driveRobotCentric(double theta, double speed, double turn) {
        driveFieldCentric(theta, speed, turn, 0.0);
    }

    /**
     * Drives based on driver pov.
     * @param gyroAngle Robot heading in radians.
     */
    public void driveFieldCentric(double gyroAngle) {
        driveFieldCentric(driveTheta, driveSpeed, turnVelocity, gyroAngle);
    }

    /**
     * Corrected driving with bias based on driver pov.
     *
     * @param theta     Direction of drive in radians.
     * @param speed     Desired driving speed in in/s.
     * @param turn      Desired angular velocity in rad/s.
     * @param gyroAngle Robot heading in radians.
     */
    public void driveFieldCentric(double theta, double speed, double turn, double gyroAngle) {
        theta = normalizeAngle(theta-gyroAngle);
        double maxSpeed = Math.hypot(
                DriveConstants.MAX_STRAFE_SPEED*Math.cos(theta),
                DriveConstants.MAX_FORWARD_SPEED*Math.sin(theta)
        );

        double L = 0;
        double R = 0;
        double theta_w = DriveConstants.THETA_WHEEL;
        if (0<theta && theta<=Math.PI/2) {
            L = 1;
            R = -Math.sin(theta_w-theta) / Math.sin(theta_w+theta);
        }
        else if (Math.PI/2<theta && theta<=Math.PI) {
            L = -Math.sin(theta_w+theta) / Math.sin(theta_w-theta);
            R = 1;
        }
        else if (-Math.PI<theta && theta<=-Math.PI/2) {
            L = -1;
            R = Math.sin(theta_w-theta) / Math.sin(theta_w+theta);
        }
        else if (-Math.PI/2<theta && theta<=0) {
            L = Math.sin(theta_w+theta) / Math.sin(theta_w-theta);
            R = -1;
        }

        double factor = speed / maxSpeed;
        L *= factor;
        R *= factor;

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = L;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = R;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = R;
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = L;

        turn /= DriveConstants.MAX_ANGULAR_VELOCITY;
        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] -= turn;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] += turn;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] -= turn;
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] += turn;

        normalize(wheelSpeeds);

        controller.driveWithMotorPowers(
                wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value],
                wheelSpeeds[RobotDrive.MotorType.kFrontRight.value],
                wheelSpeeds[RobotDrive.MotorType.kBackLeft.value],
                wheelSpeeds[RobotDrive.MotorType.kBackRight.value]
        );
    }

    /**
     * Stop the motors.
     */
    public void stopController() {
        controller.stop();
    }


    /**
     * Normalizes a given angle to (-pi,pi] radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private static double normalizeAngle(double radians) {
        return (radians + Math.PI) % (2*Math.PI) - Math.PI;
    }

    /**
     * Normalize the wheel speeds
     */
    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }

    }

}