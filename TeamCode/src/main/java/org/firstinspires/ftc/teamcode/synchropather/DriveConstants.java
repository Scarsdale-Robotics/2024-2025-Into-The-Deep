package org.firstinspires.ftc.teamcode.synchropather;


/**
 * A static class used by the robot to reference important kinematic and algorithmic tuning values.
 */
public final class DriveConstants {

	/**
	 *  Max velocity of the robot in in/s.
	 */
	public static final double MAX_VELOCITY = 54d;
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
