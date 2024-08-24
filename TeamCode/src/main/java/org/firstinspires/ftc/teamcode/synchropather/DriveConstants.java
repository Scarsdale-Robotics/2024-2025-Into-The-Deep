package org.firstinspires.ftc.teamcode.synchropather;


/**
 * A static class used by the robot to reference important kinematic and algorithmic tuning values.
 */
public final class DriveConstants {


	/**
	 *  Max velocity driving forward in in/s.
	 */
	public static final double MAX_FORWARD_SPEED = 54d;
	/**
	 * Max velocity driving sideways in in/s.
	 */
	public static final double MAX_STRAFE_SPEED = 54d;
	/**
	 * Angle of the output force vector of the robot's Mecanum wheels in radians.
	 */
	public static final double THETA_WHEEL = Math.atan2(Math.abs(MAX_FORWARD_SPEED), Math.abs(MAX_STRAFE_SPEED));


	/**
	 *  Max velocity of the robot used for SynchroPather in in/s.
	 */
	public static final double MAX_VELOCITY = Math.min(MAX_FORWARD_SPEED, MAX_STRAFE_SPEED);
	/**
	 *  Max acceleration of the robot in in/s^2.
	 */
	public static final double MAX_ACCELERATION = 54d;

	/**
	 *  Max angular velocity of the robot in rad/s.
	 */
	public static final double MAX_ANGULAR_VELOCITY = 4;
	/**
	 *  Max angular acceleration of the robot in rad/s^2.
	 */
	public static final double MAX_ANGULAR_ACCELERATION = 4;

	/**
	 *  The lookahead distance of the follower program in seconds.
	 */
	public static final double LOOKAHEAD = 0.2;

	/**
	 *  Used for differentiating and integrating spline paths, between 0 and 1 (lower = more calculations, more detail).
	 */
	public static final double delta_t = 0.005;
	
}
