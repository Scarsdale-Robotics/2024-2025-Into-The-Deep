package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles;


import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Calculates position based on elapsed time from a velocity curve defined by displacement, time and adjusted max velocity, and max acceleration.
 */
public class SymmetricMotionProfile1D extends MotionProfile1D {

	private double distance, duration, sign;
	private double v_max, a_max;

	private double minDuration;
	private TimeSpan timeSpan;

	/**
	 * Creates a new SymmetricMotionProfile1D with a given target, timeSpan, and kinematic constraints.
	 * @param targetDisplacement units
	 * @param timeSpan
	 * @param v_max units/second
	 * @param a_max units/second^2
	 */
	public SymmetricMotionProfile1D(double targetDisplacement, TimeSpan timeSpan, double v_max, double a_max) {
		this.sign = Math.signum(targetDisplacement);
		this.distance = Math.abs(targetDisplacement);
		this.v_max = v_max;
		this.a_max = a_max;
		this.timeSpan = timeSpan;
		this.duration = timeSpan.getDuration();
		init();
	}

	/**
	 * @return the absolute value of the target displacement.
	 */
	public double getTotalDistance() {
		return distance;
	}

	/**
	 * @return the target displacement.
	 */
	public double getTotalDisplacement() {
		return distance * sign;
	}

	/**
	 * @return the timestamp for when the Movement starts.
	 */
	public double getStartTime() {
		return timeSpan.getStartTime();
	}

	/**
	 * @return the timestamp for when the Movement ends.
	 */
	public double getEndTime() {
		return timeSpan.getEndTime();
	}

	/**
	 * @return the minimum time needed to reach the target displacement value.
	 */
	public double getMinDuration() {
		return minDuration;
	}

	/**
	 * @return the user-set time needed to reach the target displacement value.
	 */
	public double getDuration() {
		return timeSpan.getDuration();
	}

	/**
	 * Sets a new duration and finds max velocity accordingly.
	 */
	public void setTimeSpan(TimeSpan newTimeSpan) {
		timeSpan = newTimeSpan;

		/// catch error for when time < min_time
		if (getDuration() - minDuration < -1e-3) {
			throw new RuntimeException(
					String.format("TimeSpan duration %s is less than the minimum needed time %s.",
							getDuration(),
							minDuration
					)
			);
		}

		// floating point error correction
		if (getDuration() < minDuration) {
			timeSpan = new TimeSpan(getStartTime(), getStartTime() + minDuration);
		}

		/// calculate v_max
		// we now know that time >= min_time, so we might need to stretch the graph
		// we use quadratic formula to find v_max (minus root)
		double a, b, c, discriminant;
		a = 1/a_max;
		b = -getDuration();
		c = distance;
		// clip to prevent floating point error and ensure d >= 0
		discriminant = Math.max(0, b*b - 4*a*c);
		v_max = (-b - Math.sqrt(discriminant))/(2*a);

	}

	/**
	 * Calculates the displacement at a certain elapsed time.
	 * @param elapsedTime seconds
	 * @return the displacement value the given elapsed time.
	 */
	public double getDisplacement(double elapsedTime) {
		elapsedTime = bound(elapsedTime-getStartTime(), 0, getDuration());

		double D = Math.abs(this.distance);
		double displacement;

		double t_n = getDuration() - elapsedTime, t_a = v_max/a_max;
		if (getDuration() <= 2*t_a) {
			// triangle graph
			if (elapsedTime <= getDuration()/2)
				displacement = 0.5*a_max*elapsedTime*elapsedTime;
			else
				displacement = D - 0.5*a_max*t_n*t_n;
		}
		else {
			// trapezoid graph
			if (elapsedTime <= getDuration()/2)
				displacement = 0.5*(elapsedTime + Math.max(0, elapsedTime - t_a))* Math.min(v_max, a_max*elapsedTime);
			else
				displacement = D - 0.5*(t_n + Math.max(0, t_n - t_a))* Math.min(v_max, a_max*t_n);
		}

		displacement *= sign;

		return displacement;
	}

	/**
	 * Calculates the velocity at a certain elapsed time.
	 * @param elapsedTime seconds
	 * @return the velocity value the given elapsed time.
	 */
	public double getVelocity(double elapsedTime) {
		if (distance == 0) return 0;
		elapsedTime = bound(elapsedTime-getStartTime(), 0, getDuration());

		double velocity;

		double t_n = getDuration() - elapsedTime, t_a = v_max/a_max;
		if (getDuration() <= 2*t_a) {
			// triangle graph
			if (elapsedTime <= getDuration()/2)
				velocity = a_max*elapsedTime;
			else
				velocity = a_max*t_n;
		}
		else {
			// trapezoid graph
			if (elapsedTime <= getDuration()/2)
				velocity = Math.min(v_max, a_max*elapsedTime);
			else
				velocity = Math.min(v_max, a_max*t_n);
		}

		velocity *= sign;

		return velocity;
	}

	/**
	 * Calculates the acceleration at a certain elapsed time.
	 * @param elapsedTime seconds
	 * @return the acceleration value the given elapsed time.
	 */
	public double getAcceleration(double elapsedTime) {
		if (distance == 0) return 0;
		// Zero acceleration if outside of TimeSpan.
		if (elapsedTime-getStartTime() < 0 || getDuration() < elapsedTime-getStartTime()) return 0;

		elapsedTime = bound(elapsedTime-getStartTime(), 0, getDuration());

		double acceleration;

		double t_n = getDuration() - elapsedTime, t_a = v_max/a_max;
		if (getDuration() <= 2*t_a) {
			// triangle graph
			if (elapsedTime <= getDuration()/2)
				acceleration = a_max;
			else
				acceleration = -a_max;
		}
		else {
			// trapezoid graph
			if (elapsedTime <= t_a)
				acceleration = a_max;
			else if (t_n <= t_a)
				acceleration = -a_max;
			else
				acceleration = 0;
		}

		acceleration *= sign;

		return acceleration;
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
	 * Calculates min time of a symmetric motion profile with the given parameters.
	 */
	public static double findMinDuration(double distance, double v_max, double a_max) {
		double d_a = 0.5 * v_max*v_max / a_max;
		if (distance / 2 <= d_a) {
			// triangle graph
			return 2*Math.sqrt(distance/a_max);
		} else {
			// trapezoid graph
			return distance/v_max + v_max/a_max;
		}
	}

	/**
	 * Calculates min time and max velocity.
	 */
	public void init() {
		minDuration = findMinDuration(distance, v_max, a_max);
		setTimeSpan(timeSpan);
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
