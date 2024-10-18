package org.firstinspires.ftc.teamcode.synchropather.systems.rotation;

import org.firstinspires.ftc.teamcode.synchropather.DriveConstants;

/**
 * A static class used by Rotation Movements to reference important kinematic and algorithmic tuning values.
 */
public class RotationConstants {

    /**
     *  Max angular velocity of the robot in rad/s.
     */
    public static final double MAX_ANGULAR_VELOCITY = 0.65*DriveConstants.MAX_ANGULAR_VELOCITY; //TODO: TUNE
    /**
     *  Max angular acceleration of the robot in rad/s^2.
     */
    public static final double MAX_ANGULAR_ACCELERATION = 0.65*DriveConstants.MAX_ANGULAR_ACCELERATION; //TODO: TUNE

}