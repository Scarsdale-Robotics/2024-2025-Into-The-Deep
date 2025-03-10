package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles;


import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Calculates position based on elapsed time from a velocity curve defined by displacement, time and adjusted max velocity, and max acceleration.
 */
public class SymmetricMotionProfile1D extends MotionProfile1D {

	private final DynamicMotionProfile1D dynamicMotionProfile;

	private final double distance, sign, duration;

	private final double v_max, a_max;

	/**
	 * Creates a new SymmetricMotionProfile1D with a given target, startTime, and kinematic constraints.
	 * @param targetDisplacement units
	 * @param startTime seconds
	 * @param v_max units/second
	 * @param a_max units/second^2
	 */
	public SymmetricMotionProfile1D(double targetDisplacement, double startTime, double v_max, double a_max) {
		this.dynamicMotionProfile = new DynamicMotionProfile1D(
				targetDisplacement,
				startTime,
				0,
				v_max,
				a_max,
				a_max
		);
		this.sign = Math.signum(targetDisplacement);
		this.distance = Math.abs(targetDisplacement);
		this.v_max = Math.abs(v_max);
		this.a_max = Math.abs(a_max);
		this.duration = dynamicMotionProfile.getDuration();
	}

	/**
	 * Creates a new SymmetricMotionProfile1D with a given target, timeSpan, and kinematic constraints.
	 * @param targetDisplacement units
	 * @param timeSpan range of seconds
	 * @param v_max units/second
	 * @param a_max units/second^2
	 */
	public SymmetricMotionProfile1D(double targetDisplacement, TimeSpan timeSpan, double v_max, double a_max) {
		this.dynamicMotionProfile = new DynamicMotionProfile1D(
				targetDisplacement,
				timeSpan.getStartTime(),
				0,
				v_max,
				a_max,
				a_max
		);
		this.dynamicMotionProfile.setTimeSpan(timeSpan);
		this.sign = Math.signum(targetDisplacement);
		this.distance = Math.abs(targetDisplacement);
		this.v_max = Math.abs(v_max);
		this.a_max = Math.abs(a_max);
		this.duration = dynamicMotionProfile.getDuration();
	}

	/**
	 * @return the TimeSpan of this motion profile.
	 */
	public TimeSpan getTimeSpan() {
		return dynamicMotionProfile.getTimeSpan();
	}

	/**
	 * @return the timestamp for when the Movement starts.
	 */
	public double getStartTime() {
		return dynamicMotionProfile.getStartTime();
	}

	/**
	 * @return the timestamp for when the Movement ends.
	 */
	public double getEndTime() {
		return dynamicMotionProfile.getEndTime();
	}

	/**
	 * @return the minimum time needed to reach the target displacement value.
	 */
	public double getMinDuration() {
		return dynamicMotionProfile.getMinDuration();
	}

	/**
	 * @return the user-set time needed to reach the target displacement value.
	 */
	public double getDuration() {
		return dynamicMotionProfile.getDuration();
	}

	/**
	 * Calculates the displacement at a certain elapsed time.
	 * @param elapsedTime seconds
	 * @return the displacement value the given elapsed time.
	 */
	public double getDisplacement(double elapsedTime) {
		return dynamicMotionProfile.getDisplacement(elapsedTime);
	}

	/**
	 * Calculates the velocity at a certain elapsed time.
	 * @param elapsedTime seconds
	 * @return the velocity value the given elapsed time.
	 */
	public double getVelocity(double elapsedTime) {
		return dynamicMotionProfile.getVelocity(elapsedTime);
	}

	/**
	 * Calculates the acceleration at a certain elapsed time.
	 * @param elapsedTime seconds
	 * @return the acceleration value the given elapsed time.
	 */
	public double getAcceleration(double elapsedTime) {
		return dynamicMotionProfile.getAcceleration(elapsedTime);
	}

	/**
	 * Calculates the elapsed time at the given signed displacement value.
	 * @param displacement units
	 * @return the elapsed time at which the calculator has reached the given displacement value.
	 */
	public double getElapsedTime(double displacement) {
		displacement = bound(displacement * sign, 0, distance); // positive in [0, distance]

		double elapsedTime;
		double d_n = distance - displacement, d_a = 0.5 * v_max*v_max/a_max, t_a = v_max/a_max;
		if (duration <= 2*t_a) {
			// triangle graph
			if (displacement <= distance/2)
				elapsedTime = Math.sqrt(2*displacement/a_max);
			else
				elapsedTime = duration - Math.sqrt(2*d_n/a_max);
		}
		else {
			// trapezoid graph
			if (displacement <= distance/2)
				if (displacement <= d_a)
					// in first slope
					elapsedTime = Math.sqrt(2*displacement/a_max);
				else
					// in first plateau
					elapsedTime = displacement/v_max + t_a/2d;
			else
			if (d_n <= d_a)
				// in last slope
				elapsedTime = duration - Math.sqrt(2*d_n/a_max);
			else
				// in second plateau
				elapsedTime = duration - (d_n/v_max + t_a/2d);
		}

		return elapsedTime;
	}

	/**
	 * Clips the input x between a given lower and upper bound.
	 * @param x
	 * @param lower
	 * @param upper
	 * @return the clipped value of x.
	 */
	private static double bound(double x, double lower, double upper) {
		return Math.max(lower, Math.min(upper, x));
	}

}
