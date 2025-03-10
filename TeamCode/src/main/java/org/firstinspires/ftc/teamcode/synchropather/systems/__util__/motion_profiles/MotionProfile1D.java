package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Calculates position based on elapsed time from a velocity curve.
 */
public abstract class MotionProfile1D {

	/**
	 * @return the TimeSpan of this motion profile.
	 */
	public abstract TimeSpan getTimeSpan();

	/**
	 * @return the timestamp for when the Movement starts.
	 */
	public abstract double getStartTime();

	/**
	 * @return the timestamp for when the Movement ends.
	 */
	public abstract double getEndTime();

	/**
	 * @return the time needed to reach the target displacement value in seconds
	 */
	public abstract double getDuration();

	/**
	 * @return the minimum time needed to reach the target displacement value.
	 */
	public abstract double getMinDuration();

	/**
	 * Calculates the displacement at a certain elapsed time.
	 * @param elapsedTime 
	 * @return the displacement value the given elapsed time.
	 */
	public abstract double getDisplacement(double elapsedTime);

	/**
	 * Calculates the velocity at a certain elapsed time.
	 * @param elapsedTime
	 * @return the velocity value the given elapsed time.
	 */
	public abstract double getVelocity(double elapsedTime);

	/**
	 * Calculates the acceleration at a certain elapsed time.
	 * @param elapsedTime
	 * @return the acceleration value the given elapsed time.
	 */
	public abstract double getAcceleration(double elapsedTime);
	
}