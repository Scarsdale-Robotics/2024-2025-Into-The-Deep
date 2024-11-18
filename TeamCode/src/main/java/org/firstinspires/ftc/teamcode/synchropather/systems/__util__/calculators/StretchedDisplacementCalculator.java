package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.calculators;


import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Object that calculates position based on elapsed time from a velocity curve defined by displacement, time and adjusted max velocity, and max acceleration.
 */
public class StretchedDisplacementCalculator extends DisplacementCalculator {

	private double minDuration;
	private TimeSpan timeSpan;

	/**
	 * Creates a new StretchedDisplacementCalculator with a given target, timeSpan, and kinematic constraints.
	 * @param targetDisplacement
	 * @param timeSpan
	 * @param MV
	 * @param MA
	 */
	public StretchedDisplacementCalculator(double targetDisplacement, TimeSpan timeSpan, double MV, double MA) {
		super(targetDisplacement, MV, MA);
		this.timeSpan = timeSpan;
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

		/// calculate MV
		// we now know that time >= min_time, so we might need to stretch the graph
		// we use quadratic formula to find MV (minus root)
		double a, b, c, discriminant;
		a = 1/MA;
		b = -getDuration();
		c = distance;
		// clip to prevent floating point error and ensure d >= 0
		discriminant = Math.max(0, b*b - 4*a*c);
		MV = (-b - Math.sqrt(discriminant))/(2*a);

	}

	/**
	 * Calculates the displacement at a certain elapsed time.
	 * @param elapsedTime
	 * @return the displacement value the given elapsed time.
	 */
	public double getDisplacement(double elapsedTime) {
		elapsedTime = bound(elapsedTime-getStartTime(), 0, getDuration());

		double D = Math.abs(this.distance);
		double displacement;

		double t_n = getDuration() - elapsedTime, t_a = MV/MA;
		if (getDuration() <= 2*t_a) {
			// triangle graph
			if (elapsedTime <= getDuration()/2)
				displacement = 0.5*MA*elapsedTime*elapsedTime;
			else
				displacement = D - 0.5*MA*t_n*t_n;
		}
		else {
			// trapezoid graph
			if (elapsedTime <= getDuration()/2)
				displacement = 0.5*(elapsedTime + Math.max(0, elapsedTime - t_a))* Math.min(MV, MA*elapsedTime);
			else
				displacement = D - 0.5*(t_n + Math.max(0, t_n - t_a))* Math.min(MV, MA*t_n);
		}

		displacement *= sign;

		return displacement;
	}

	/**
	 * Calculates the velocity at a certain elapsed time.
	 * @param elapsedTime
	 * @return the velocity value the given elapsed time.
	 */
	public double getVelocity(double elapsedTime) {
		if (distance == 0) return 0;
		elapsedTime = bound(elapsedTime-getStartTime(), 0, getDuration());

		double velocity;

		double t_n = getDuration() - elapsedTime, t_a = MV/MA;
		if (getDuration() <= 2*t_a) {
			// triangle graph
			if (elapsedTime <= getDuration()/2)
				velocity = MA*elapsedTime;
			else
				velocity = MA*t_n;
		}
		else {
			// trapezoid graph
			if (elapsedTime <= getDuration()/2)
				velocity = Math.min(MV, MA*elapsedTime);
			else
				velocity = Math.min(MV, MA*t_n);
		}

		velocity *= sign;

		return velocity;
	}

	/**
	 * Calculates the acceleration at a certain elapsed time.
	 * @param elapsedTime
	 * @return the acceleration value the given elapsed time.
	 */
	public double getAcceleration(double elapsedTime) {
		if (distance == 0) return 0;
		elapsedTime = bound(elapsedTime-getStartTime(), 0, getDuration());

		double acceleration;

		double t_n = getDuration() - elapsedTime, t_a = MV/MA;
		if (getDuration() <= 2*t_a) {
			// triangle graph
			if (elapsedTime <= getDuration()/2)
				acceleration = MA;
			else
				acceleration = -MA;
		}
		else {
			// trapezoid graph
			if (elapsedTime <= t_a)
				acceleration = MA;
			else if (t_n <= t_a)
				acceleration = -MA;
			else
				acceleration = 0;
		}

		acceleration *= sign;

		return acceleration;
	}

	public static double findMinDuration(double distance, double MV, double MA) {
		double d_a = 0.5 * MV*MV / MA;
		if (distance / 2 <= d_a) {
			// triangle graph
			return 2*Math.sqrt(distance/MA);
		} else {
			// trapezoid graph
			return distance/MV + MV/MA;
		}
	}

	/**
	 * Calculates min time and max velocity.
	 */
	public void init() {
		minDuration = findMinDuration(distance, MV, MA);
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
