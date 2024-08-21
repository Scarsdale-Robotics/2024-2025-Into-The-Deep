package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.calculators;

/**
 * Object that calculates position based on elapsed time from a velocity curve defined by displacement, time, max velocity, and max acceleration.
 */
public class BoundedDisplacementCalculator extends DisplacementCalculator {

	private double distance, duration, sign;
	private double MV, MA;

	/**
	 * Creates a new BoundedDisplacementCalculator with a given target, duration, and kinematic constraints.
	 * @param targetDisplacement
	 * @param targetDuration
	 * @param MV
	 * @param MA
	 */
	public BoundedDisplacementCalculator(double targetDisplacement, double targetDuration, double MV, double MA) {
		super(targetDisplacement, MV, MA);
		this.duration = targetDuration;
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
	 * @return the minimum time needed to reach the target displacement value.
	 */
	public double getDuration() {
		return duration;
	}

	/**
	 * Calculates the displacement at a certain elapsed time.
	 * @param elapsedTime 
	 * @return the displacement value the given elapsed time.
	 */
	public double getDisplacement(double elapsedTime) {
		elapsedTime = bound(elapsedTime, 0, duration);
		
		double D = Math.abs(this.distance);
		double displacement;
		
		double t_n = duration - elapsedTime, t_a = MV/MA;
		if (duration <= 2*t_a) {
			// triangle graph
			if (elapsedTime <= duration/2)
				displacement = 0.5*MA*elapsedTime*elapsedTime;
			else
				displacement = D - 0.5*MA*t_n*t_n;
		} 
		else {
			// trapezoid graph
			if (elapsedTime <= duration/2)
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
		elapsedTime = bound(elapsedTime, 0, duration);
		
		double velocity;
		
		double t_n = duration - elapsedTime, t_a = MV/MA;
		if (duration <= 2*t_a) {
			// triangle graph
			if (elapsedTime <= duration/2)
				velocity = MA*elapsedTime;
			else
				velocity = MA*t_n;
		} 
		else {
			// trapezoid graph
			if (elapsedTime <= duration/2)
				velocity = Math.min(MV, MA*elapsedTime);
			else
				velocity = Math.min(MV, MA*t_n);
		}
		
		velocity *= sign;
		
		return velocity;
	}

	/**
	 * Calculates max distance, min time, and max velocity.
	 */
	public void init() {
		
		double T = duration;
		double t_a = MV/MA;
		double D_max;
		
		if (T < 2*t_a) {
			// triangle graph
			D_max = MA * T * T / 4.0;
		} 
		else {
			// trapezoid graph
			D_max = (T - MV/MA) * MV;
		}
		
		if (distance >= D_max) {
			// no need to compensate MV
			distance = D_max;
		}

		// trim time
		double previousMV = MV;
		MV = Math.min(MV, Math.sqrt(distance*MA));
		
		if (Math.abs(MV-previousMV)<1e-3)
			duration = distance/MV + t_a;
		else
			duration = Math.sqrt(4*distance/MA);
		
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
